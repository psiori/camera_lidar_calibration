#include "psiviz/point_cloud_renderer.hpp"

#include "psiviz/depth_picker.hpp"
#include "psiviz/scene_shapes.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

#include <QColor>
#include <QDebug>
#include <QFile>
#include <QMatrix4x4>
#include <QRect>
#include <rhi/qrhi.h>
#include <rhi/qshader.h>

namespace psiviz {

namespace {

struct GpuVertex {
  float px, py, pz;
  float scalar;
  float r, g, b;
};

struct UniformBlock {
  float mvp[16];
  float pointSize;
  float blendWeight;
  float scalarMin;
  float scalarMax;
};

struct GizmoVertex {
  float px, py, pz;
  float r, g, b;
};

struct GizmoUniform {
  float mvp[16];
};

QShader loadShader(const QString& name) {
  QFile file(name);
  if (!file.open(QIODevice::ReadOnly)) {
    return {};
  }
  return QShader::fromSerialized(file.readAll());
}

QShader loadFirst(const QStringList& names) {
  for (const auto& n : names) {
    QShader s = loadShader(n);
    if (s.isValid()) {
      return s;
    }
  }
  return {};
}

void appendAxis(std::vector<GizmoVertex>& v, const Eigen::Vector3f& o, const Eigen::Vector3f& dir, float len,
                float r, float g, float b) {
  const Eigen::Vector3f e = o + dir * len;
  v.push_back({o.x(), o.y(), o.z(), r, g, b});
  v.push_back({e.x(), e.y(), e.z(), r, g, b});
}

void appendRing(std::vector<GizmoVertex>& v, const Eigen::Vector3f& o, const Eigen::Matrix3f& R, int axis, float radius,
                float r, float g, float b) {
  constexpr int kSeg = 48;
  Eigen::Vector3f u, w;
  if (axis == 0) {
    u = R.col(1);
    w = R.col(2);
  } else if (axis == 1) {
    u = R.col(2);
    w = R.col(0);
  } else {
    u = R.col(0);
    w = R.col(1);
  }
  Eigen::Vector3f prev = o + u * radius;
  for (int i = 1; i <= kSeg; ++i) {
    const float a = static_cast<float>(i) / static_cast<float>(kSeg) * 6.2831853f;
    const Eigen::Vector3f p = o + (u * std::cos(a) + w * std::sin(a)) * radius;
    v.push_back({prev.x(), prev.y(), prev.z(), r, g, b});
    v.push_back({p.x(), p.y(), p.z(), r, g, b});
    prev = p;
  }
}

}  // namespace

struct PointCloudRenderer::RhiResources {
  std::unique_ptr<QRhiBuffer> vertex_buffer;
  std::unique_ptr<QRhiBuffer> uniform_buffer;
  std::unique_ptr<QRhiGraphicsPipeline> pipeline;
  std::unique_ptr<QRhiShaderResourceBindings> srb;

  std::unique_ptr<QRhiBuffer> gizmo_vertex_buffer;
  std::unique_ptr<QRhiBuffer> gizmo_uniform_buffer;
  std::unique_ptr<QRhiGraphicsPipeline> gizmo_pipeline;
  std::unique_ptr<QRhiShaderResourceBindings> gizmo_srb;

  std::unique_ptr<QRhiBuffer> shape_vertex_buffer;
  std::unique_ptr<QRhiBuffer> shape_uniform_buffer;
  std::unique_ptr<QRhiGraphicsPipeline> shape_pipeline;
  std::unique_ptr<QRhiShaderResourceBindings> shape_srb;

  std::unique_ptr<QRhiTexture> pick_color;
  std::unique_ptr<QRhiRenderBuffer> pick_depth;
  std::unique_ptr<QRhiTextureRenderTarget> pick_rt;
  std::unique_ptr<QRhiRenderPassDescriptor> pick_rp;
  std::unique_ptr<QRhiGraphicsPipeline> pick_pipeline;
  QSize pick_size;

  struct PickReadback {
    QRhiReadbackResult result;
    bool in_flight{false};
    bool completed{false};
    std::uint64_t request_id{0};
    int patch_center_x{0};
    int patch_center_y{0};
    int patch_w{0};
    int patch_h{0};
    bool flip_framebuffer_y{false};
  };
  PickReadback pick_readback;
};

PointCloudRenderer::PointCloudRenderer(QRhi* rhi) : rhi_(rhi), rhi_resources_(std::make_unique<RhiResources>()) {}

PointCloudRenderer::~PointCloudRenderer() {
  releaseResources();
}

void PointCloudRenderer::releaseResources() {
  if (rhi_resources_) {
    rhi_resources_->pick_pipeline.reset();
    rhi_resources_->pick_rt.reset();
    rhi_resources_->pick_rp.reset();
    rhi_resources_->pick_depth.reset();
    rhi_resources_->pick_color.reset();
    rhi_resources_->gizmo_pipeline.reset();
    rhi_resources_->gizmo_srb.reset();
    rhi_resources_->gizmo_uniform_buffer.reset();
    rhi_resources_->gizmo_vertex_buffer.reset();
    rhi_resources_->shape_pipeline.reset();
    rhi_resources_->shape_srb.reset();
    rhi_resources_->shape_uniform_buffer.reset();
    rhi_resources_->shape_vertex_buffer.reset();
    rhi_resources_->srb.reset();
    rhi_resources_->pipeline.reset();
    rhi_resources_->uniform_buffer.reset();
    rhi_resources_->vertex_buffer.reset();
  }
  initialized_ = false;
  pipeline_ready_ = false;
  point_count_ = 0;
  shape_vertex_count_ = 0;
}

void PointCloudRenderer::initialize(QRhiCommandBuffer* cb) {
  Q_UNUSED(cb);
  initialized_ = true;
}

void PointCloudRenderer::resize(int width, int height) {
  width_ = width;
  height_ = height;
}

std::uint64_t PointCloudRenderer::requestPick(int x, int y, float dpr, float point_size) {
  PickRequest req;
  req.request_id = next_request_id_++;
  req.x = x;
  req.y = y;
  req.dpr = dpr;
  req.point_size = point_size;

  auto& rb = rhi_resources_->pick_readback;
  if (rb.in_flight) {
    queued_request_ = req;
  } else {
    active_request_ = req;
    pick_pass_scheduled_ = true;
  }
  return req.request_id;
}

void PointCloudRenderer::pollPickReadback() {
  if (!rhi_resources_) {
    return;
  }
  auto& rb = rhi_resources_->pick_readback;
  if (!rb.in_flight || !rb.completed) {
    return;
  }
  std::uint32_t point_id = 0;
  if (!rb.result.data.isEmpty() && rb.patch_w > 0 && rb.patch_h > 0) {
    const auto* pixels = reinterpret_cast<const unsigned char*>(rb.result.data.constData());
    point_id = DepthPicker::selectPointIdFromRgbaPatch(pixels, rb.patch_w, rb.patch_h, rb.patch_center_x,
                                                       rb.patch_center_y, rb.flip_framebuffer_y);
  }
  completed_picks_.push_back({rb.request_id, point_id});

  rb.in_flight = false;
  rb.completed = false;
  active_request_.reset();
  schedulePickPassIfIdle();
}

bool PointCloudRenderer::tryConsumePickResult(std::uint64_t request_id, std::uint32_t& point_id) {
  pollPickReadback();
  const auto it = std::find_if(completed_picks_.begin(), completed_picks_.end(),
                               [request_id](const auto& entry) { return entry.first == request_id; });
  if (it == completed_picks_.end()) {
    return false;
  }
  point_id = it->second;
  completed_picks_.erase(it);
  return true;
}

void PointCloudRenderer::schedulePickPassIfIdle() {
  if (!rhi_resources_ || rhi_resources_->pick_readback.in_flight || pick_pass_scheduled_) {
    return;
  }
  if (!queued_request_) {
    return;
  }
  active_request_ = queued_request_;
  queued_request_.reset();
  pick_pass_scheduled_ = true;
}

void PointCloudRenderer::ensurePipeline(QRhiRenderTarget* rt) {
  auto& res = *rhi_resources_;
  if (!rhi_) {
    return;
  }

  QShader vs = loadFirst({QStringLiteral(":/psiviz/shaders/pointcloud.vert.qsb"),
                          QStringLiteral(":/psiviz/shaders/shaders/pointcloud.vert.qsb")});
  QShader fs = loadFirst({QStringLiteral(":/psiviz/shaders/pointcloud.frag.qsb"),
                          QStringLiteral(":/psiviz/shaders/shaders/pointcloud.frag.qsb")});
  if (!vs.isValid() || !fs.isValid()) {
    static bool warned = false;
    if (!warned) {
      qWarning("psiviz: point cloud shaders failed to load (vert=%d frag=%d)", vs.isValid(), fs.isValid());
      warned = true;
    }
    return;
  }

  if (!res.uniform_buffer) {
    res.uniform_buffer.reset(rhi_->newBuffer(QRhiBuffer::Dynamic, QRhiBuffer::UniformBuffer, sizeof(UniformBlock)));
    res.uniform_buffer->create();
  }
  if (!res.vertex_buffer) {
    res.vertex_buffer.reset(rhi_->newBuffer(QRhiBuffer::Dynamic, QRhiBuffer::VertexBuffer, 0));
    res.vertex_buffer->create();
  }
  if (!res.srb) {
    res.srb.reset(rhi_->newShaderResourceBindings());
    res.srb->setBindings({
        QRhiShaderResourceBinding::uniformBuffer(0, QRhiShaderResourceBinding::VertexStage | QRhiShaderResourceBinding::FragmentStage,
                                                 res.uniform_buffer.get()),
    });
    res.srb->create();
  }
  if (!res.pipeline) {
    res.pipeline.reset(rhi_->newGraphicsPipeline());
    res.pipeline->setShaderStages({{QRhiShaderStage::Vertex, vs}, {QRhiShaderStage::Fragment, fs}});
    QRhiVertexInputLayout input;
    input.setBindings({{sizeof(GpuVertex)}});
    input.setAttributes({
        {0, 0, QRhiVertexInputAttribute::Float3, 0},
        {0, 1, QRhiVertexInputAttribute::Float, offsetof(GpuVertex, scalar)},
        {0, 2, QRhiVertexInputAttribute::Float3, offsetof(GpuVertex, r)},
    });
    res.pipeline->setVertexInputLayout(input);
    res.pipeline->setShaderResourceBindings(res.srb.get());
    res.pipeline->setTopology(QRhiGraphicsPipeline::Points);
    res.pipeline->setDepthTest(true);
    res.pipeline->setDepthWrite(true);
    res.pipeline->setDepthOp(QRhiGraphicsPipeline::Less);
  }
  if (rt && res.pipeline) {
    QRhiRenderPassDescriptor* const pass = rt->renderPassDescriptor();
    if (res.pipeline->renderPassDescriptor() != pass || !pipeline_ready_) {
      res.pipeline->setRenderPassDescriptor(pass);
      pipeline_ready_ = res.pipeline->create();
      if (!pipeline_ready_) {
        static bool warned = false;
        if (!warned) {
          qWarning("psiviz: point cloud pipeline creation failed");
          warned = true;
        }
      }
    }
  }
}

void PointCloudRenderer::ensureGizmoPipeline(QRhiRenderTarget* rt) {
  auto& res = *rhi_resources_;
  if (!rhi_ || !rt) {
    return;
  }
  QShader vs = loadFirst({QStringLiteral(":/psiviz/shaders/gizmo.vert.qsb"),
                          QStringLiteral(":/psiviz/shaders/shaders/gizmo.vert.qsb")});
  QShader fs = loadFirst({QStringLiteral(":/psiviz/shaders/gizmo.frag.qsb"),
                          QStringLiteral(":/psiviz/shaders/shaders/gizmo.frag.qsb")});
  if (!vs.isValid() || !fs.isValid()) {
    return;
  }
  if (!res.gizmo_uniform_buffer) {
    res.gizmo_uniform_buffer.reset(rhi_->newBuffer(QRhiBuffer::Dynamic, QRhiBuffer::UniformBuffer, sizeof(GizmoUniform)));
    res.gizmo_uniform_buffer->create();
  }
  if (!res.gizmo_vertex_buffer) {
    res.gizmo_vertex_buffer.reset(rhi_->newBuffer(QRhiBuffer::Dynamic, QRhiBuffer::VertexBuffer, 4096));
    res.gizmo_vertex_buffer->create();
  }
  if (!res.gizmo_srb) {
    res.gizmo_srb.reset(rhi_->newShaderResourceBindings());
    res.gizmo_srb->setBindings({QRhiShaderResourceBinding::uniformBuffer(
        0, QRhiShaderResourceBinding::VertexStage | QRhiShaderResourceBinding::FragmentStage, res.gizmo_uniform_buffer.get())});
    res.gizmo_srb->create();
  }
  if (!res.gizmo_pipeline) {
    res.gizmo_pipeline.reset(rhi_->newGraphicsPipeline());
    res.gizmo_pipeline->setShaderStages({{QRhiShaderStage::Vertex, vs}, {QRhiShaderStage::Fragment, fs}});
    QRhiVertexInputLayout input;
    input.setBindings({{sizeof(GizmoVertex)}});
    input.setAttributes({
        {0, 0, QRhiVertexInputAttribute::Float3, 0},
        {0, 1, QRhiVertexInputAttribute::Float3, offsetof(GizmoVertex, r)},
    });
    res.gizmo_pipeline->setVertexInputLayout(input);
    res.gizmo_pipeline->setShaderResourceBindings(res.gizmo_srb.get());
    res.gizmo_pipeline->setTopology(QRhiGraphicsPipeline::Lines);
    res.gizmo_pipeline->setDepthTest(false);
    res.gizmo_pipeline->setDepthWrite(false);
    res.gizmo_pipeline->setLineWidth(2.f);
  }
  if (res.gizmo_pipeline->renderPassDescriptor() != rt->renderPassDescriptor()) {
    res.gizmo_pipeline->setRenderPassDescriptor(rt->renderPassDescriptor());
    res.gizmo_pipeline->create();
  }
}

void PointCloudRenderer::ensureShapePipeline(QRhiRenderTarget* rt) {
  auto& res = *rhi_resources_;
  if (!rhi_ || !rt) {
    return;
  }
  QShader vs = loadFirst({QStringLiteral(":/psiviz/shaders/shape.vert.qsb"),
                          QStringLiteral(":/psiviz/shaders/shaders/shape.vert.qsb")});
  QShader fs = loadFirst({QStringLiteral(":/psiviz/shaders/shape.frag.qsb"),
                          QStringLiteral(":/psiviz/shaders/shaders/shape.frag.qsb")});
  if (!vs.isValid() || !fs.isValid()) {
    return;
  }
  if (!res.shape_uniform_buffer) {
    res.shape_uniform_buffer.reset(rhi_->newBuffer(QRhiBuffer::Dynamic, QRhiBuffer::UniformBuffer, sizeof(GizmoUniform)));
    res.shape_uniform_buffer->create();
  }
  if (!res.shape_vertex_buffer) {
    res.shape_vertex_buffer.reset(rhi_->newBuffer(QRhiBuffer::Dynamic, QRhiBuffer::VertexBuffer, 65536));
    res.shape_vertex_buffer->create();
  }
  if (!res.shape_srb) {
    res.shape_srb.reset(rhi_->newShaderResourceBindings());
    res.shape_srb->setBindings({QRhiShaderResourceBinding::uniformBuffer(
        0, QRhiShaderResourceBinding::VertexStage | QRhiShaderResourceBinding::FragmentStage,
        res.shape_uniform_buffer.get())});
    res.shape_srb->create();
  }
  if (!res.shape_pipeline) {
    res.shape_pipeline.reset(rhi_->newGraphicsPipeline());
    res.shape_pipeline->setShaderStages({{QRhiShaderStage::Vertex, vs}, {QRhiShaderStage::Fragment, fs}});
    QRhiVertexInputLayout input;
    input.setBindings({{sizeof(ShapeVertex)}});
    input.setAttributes({
        {0, 0, QRhiVertexInputAttribute::Float3, 0},
        {0, 1, QRhiVertexInputAttribute::Float3, offsetof(ShapeVertex, r)},
        {0, 2, QRhiVertexInputAttribute::Float, offsetof(ShapeVertex, a)},
    });
    res.shape_pipeline->setVertexInputLayout(input);
    res.shape_pipeline->setShaderResourceBindings(res.shape_srb.get());
    res.shape_pipeline->setTopology(QRhiGraphicsPipeline::Triangles);
    res.shape_pipeline->setDepthTest(true);
    res.shape_pipeline->setDepthWrite(false);
    res.shape_pipeline->setCullMode(QRhiGraphicsPipeline::Back);
    QRhiGraphicsPipeline::TargetBlend blend;
    blend.enable = true;
    blend.srcColor = QRhiGraphicsPipeline::SrcAlpha;
    blend.dstColor = QRhiGraphicsPipeline::OneMinusSrcAlpha;
    blend.opColor = QRhiGraphicsPipeline::Add;
    blend.srcAlpha = QRhiGraphicsPipeline::One;
    blend.dstAlpha = QRhiGraphicsPipeline::OneMinusSrcAlpha;
    blend.opAlpha = QRhiGraphicsPipeline::Add;
    res.shape_pipeline->setTargetBlends({blend});
  }
  if (res.shape_pipeline->renderPassDescriptor() != rt->renderPassDescriptor()) {
    res.shape_pipeline->setRenderPassDescriptor(rt->renderPassDescriptor());
    res.shape_pipeline->create();
  }
}

void PointCloudRenderer::setSphereMarkers(const std::vector<SphereMarker>& markers) {
  sphere_markers_ = markers;
}

void PointCloudRenderer::uploadSphereMarkers() {
  auto& res = *rhi_resources_;
  if (!res.shape_vertex_buffer) {
    shape_vertex_count_ = 0;
    return;
  }
  std::vector<ShapeVertex> verts;
  buildSphereMarkerMesh(verts, sphere_markers_);
  if (verts.empty()) {
    shape_vertex_count_ = 0;
    return;
  }
  const quint32 bytes = static_cast<quint32>(verts.size() * sizeof(ShapeVertex));
  if (res.shape_vertex_buffer->size() < bytes) {
    res.shape_vertex_buffer->setSize(bytes);
    res.shape_vertex_buffer->create();
  }
  void* ptr = res.shape_vertex_buffer->beginFullDynamicBufferUpdateForCurrentFrame();
  if (ptr) {
    std::memcpy(ptr, verts.data(), bytes);
    res.shape_vertex_buffer->endFullDynamicBufferUpdateForCurrentFrame();
  }
  shape_vertex_count_ = static_cast<int>(verts.size());
}

void PointCloudRenderer::ensurePickTarget(int width, int height) {
  auto& res = *rhi_resources_;
  if (!rhi_ || width <= 0 || height <= 0) {
    return;
  }
  const QSize size(width, height);
  if (res.pick_rt && res.pick_size == size) {
    return;
  }
  res.pick_pipeline.reset();
  res.pick_rt.reset();
  res.pick_rp.reset();
  res.pick_depth.reset();
  res.pick_color.reset();

  res.pick_color.reset(
      rhi_->newTexture(QRhiTexture::RGBA8, size, 1, QRhiTexture::RenderTarget | QRhiTexture::UsedAsTransferSource));
  res.pick_color->create();
  res.pick_depth.reset(rhi_->newRenderBuffer(QRhiRenderBuffer::DepthStencil, size));
  res.pick_depth->create();
  QRhiTextureRenderTargetDescription desc(res.pick_color.get());
  desc.setDepthStencilBuffer(res.pick_depth.get());
  res.pick_rt.reset(rhi_->newTextureRenderTarget(desc));
  res.pick_rp.reset(res.pick_rt->newCompatibleRenderPassDescriptor());
  res.pick_rt->setRenderPassDescriptor(res.pick_rp.get());
  res.pick_rt->create();
  res.pick_size = size;
  res.pick_readback = {};

  QShader vs = loadFirst({QStringLiteral(":/psiviz/shaders/pick.vert.qsb"),
                          QStringLiteral(":/psiviz/shaders/shaders/pick.vert.qsb")});
  QShader fs = loadFirst({QStringLiteral(":/psiviz/shaders/pick.frag.qsb"),
                          QStringLiteral(":/psiviz/shaders/shaders/pick.frag.qsb")});
  if (!vs.isValid() || !fs.isValid() || !res.srb) {
    return;
  }
  res.pick_pipeline.reset(rhi_->newGraphicsPipeline());
  res.pick_pipeline->setShaderStages({{QRhiShaderStage::Vertex, vs}, {QRhiShaderStage::Fragment, fs}});
  QRhiVertexInputLayout input;
  input.setBindings({{sizeof(GpuVertex)}});
  input.setAttributes({
      {0, 0, QRhiVertexInputAttribute::Float3, 0},
      {0, 1, QRhiVertexInputAttribute::Float, offsetof(GpuVertex, scalar)},
      {0, 2, QRhiVertexInputAttribute::Float3, offsetof(GpuVertex, r)},
  });
  res.pick_pipeline->setVertexInputLayout(input);
  res.pick_pipeline->setShaderResourceBindings(res.srb.get());
  res.pick_pipeline->setTopology(QRhiGraphicsPipeline::Points);
  res.pick_pipeline->setDepthTest(true);
  res.pick_pipeline->setDepthWrite(true);
  res.pick_pipeline->setDepthOp(QRhiGraphicsPipeline::Less);
  res.pick_pipeline->setRenderPassDescriptor(res.pick_rp.get());
  res.pick_pipeline->create();
}

void PointCloudRenderer::uploadGizmo(const PoseGizmo& gizmo) {
  auto& res = *rhi_resources_;
  if (!res.gizmo_vertex_buffer || !gizmo.enabled()) {
    gizmo_vertex_count_ = 0;
    return;
  }
  std::vector<GizmoVertex> verts;
  const Eigen::Vector3f o = gizmo.pose().translation();
  const Eigen::Matrix3f R = gizmo.pose().linear();
  const float s = gizmo.clipScale();
  appendAxis(verts, o, R.col(0), s, 1.f, 0.2f, 0.2f);
  appendAxis(verts, o, R.col(1), s, 0.2f, 1.f, 0.2f);
  appendAxis(verts, o, R.col(2), s, 0.3f, 0.4f, 1.f);
  appendRing(verts, o, R, 0, s * 0.7f, 1.f, 0.3f, 0.3f);
  appendRing(verts, o, R, 1, s * 0.7f, 0.3f, 1.f, 0.3f);
  appendRing(verts, o, R, 2, s * 0.7f, 0.3f, 0.4f, 1.f);

  const quint32 bytes = static_cast<quint32>(verts.size() * sizeof(GizmoVertex));
  if (res.gizmo_vertex_buffer->size() < bytes) {
    res.gizmo_vertex_buffer->setSize(bytes);
    res.gizmo_vertex_buffer->create();
  }
  void* ptr = res.gizmo_vertex_buffer->beginFullDynamicBufferUpdateForCurrentFrame();
  if (ptr) {
    std::memcpy(ptr, verts.data(), bytes);
    res.gizmo_vertex_buffer->endFullDynamicBufferUpdateForCurrentFrame();
  }
  gizmo_vertex_count_ = static_cast<int>(verts.size());
}

void PointCloudRenderer::uploadPointCloud(const PointCloudData& data, QRhiRenderTarget* rt) {
  if (!rhi_ || data.point_count == 0) {
    point_count_ = 0;
    return;
  }
  ensurePipeline(rt);
  auto& vb = rhi_resources_->vertex_buffer;
  if (!vb) {
    point_count_ = 0;
    return;
  }

  std::vector<GpuVertex> vertices(data.point_count);
  for (std::uint64_t i = 0; i < data.point_count; ++i) {
    vertices[i].px = data.positions[i * 3 + 0];
    vertices[i].py = data.positions[i * 3 + 1];
    vertices[i].pz = data.positions[i * 3 + 2];
    vertices[i].scalar = data.scalars[i];
    vertices[i].r = data.rgb[i * 3 + 0];
    vertices[i].g = data.rgb[i * 3 + 1];
    vertices[i].b = data.rgb[i * 3 + 2];
  }

  const quint32 bytes = static_cast<quint32>(vertices.size() * sizeof(GpuVertex));
  if (vb->size() < bytes) {
    vb->setSize(bytes);
    vb->create();
  }
  void* ptr = vb->beginFullDynamicBufferUpdateForCurrentFrame();
  if (!ptr) {
    qWarning("psiviz: point cloud vertex buffer update failed (count=%llu)",
             static_cast<unsigned long long>(data.point_count));
    point_count_ = 0;
    return;
  }
  std::memcpy(ptr, vertices.data(), bytes);
  vb->endFullDynamicBufferUpdateForCurrentFrame();
  point_count_ = static_cast<int>(data.point_count);
}

void PointCloudRenderer::renderPickPass(QRhiCommandBuffer* cb, const TrackballCamera& camera, int fb_w, int fb_h,
                                        const PickRequest& request) {
  auto& res = *rhi_resources_;
  if (!res.pick_rt || !res.pick_pipeline || point_count_ <= 0) {
    completed_picks_.push_back({request.request_id, 0});
    active_request_.reset();
    return;
  }

  UniformBlock ub{};
  Eigen::Matrix4f clip_corr = Eigen::Matrix4f::Identity();
  if (rhi_) {
    const QMatrix4x4 qt_corr = rhi_->clipSpaceCorrMatrix();
    std::memcpy(clip_corr.data(), qt_corr.constData(), sizeof(float) * 16);
  }
  const Eigen::Matrix4f mvp = clip_corr * camera.projectionMatrix() * camera.viewMatrix();
  std::memcpy(ub.mvp, mvp.data(), sizeof(ub.mvp));
  ub.pointSize = request.point_size;
  ub.blendWeight = blend_weight_;
  ub.scalarMin = scalar_min_;
  ub.scalarMax = scalar_max_;
  if (res.uniform_buffer) {
    void* ptr = res.uniform_buffer->beginFullDynamicBufferUpdateForCurrentFrame();
    if (ptr) {
      std::memcpy(ptr, &ub, sizeof(ub));
      res.uniform_buffer->endFullDynamicBufferUpdateForCurrentFrame();
    }
  }

  cb->beginPass(res.pick_rt.get(), QColor(Qt::black), {1.f, 0}, nullptr);
  cb->setGraphicsPipeline(res.pick_pipeline.get());
  cb->setViewport({0, 0, static_cast<float>(fb_w), static_cast<float>(fb_h)});
  cb->setShaderResources();
  const QRhiCommandBuffer::VertexInput vbuf(res.vertex_buffer.get(), 0);
  cb->setVertexInput(0, 1, &vbuf);
  cb->draw(point_count_);

  const int px =
      std::clamp(static_cast<int>(std::lround(static_cast<float>(request.x) * request.dpr)), 0, fb_w - 1);
  const int py =
      std::clamp(static_cast<int>(std::lround(static_cast<float>(request.y) * request.dpr)), 0, fb_h - 1);
  const int radius =
      std::max(2, static_cast<int>(std::ceil(request.point_size * request.dpr * 0.5f)) + 1);
  QRect read_rect(px - radius, py - radius, 2 * radius + 1, 2 * radius + 1);
  read_rect = read_rect.intersected(QRect(0, 0, fb_w, fb_h));

  auto& pick_rb = res.pick_readback;
  pick_rb.completed = false;
  pick_rb.request_id = request.request_id;
  pick_rb.patch_center_x = px - read_rect.x();
  pick_rb.patch_center_y = py - read_rect.y();
  pick_rb.patch_w = read_rect.width();
  pick_rb.patch_h = read_rect.height();
  pick_rb.flip_framebuffer_y = rhi_ && rhi_->isYUpInFramebuffer();
  pick_rb.result = QRhiReadbackResult();
  pick_rb.result.completed = [this]() {
    if (!rhi_resources_) {
      return;
    }
    rhi_resources_->pick_readback.completed = true;
  };

  QRhiReadbackDescription read_desc(res.pick_color.get());
  read_desc.setRect(read_rect);
  QRhiResourceUpdateBatch* batch = rhi_->nextResourceUpdateBatch();
  batch->readBackTexture(read_desc, &pick_rb.result);
  cb->endPass(batch);
  pick_rb.in_flight = true;
  Q_UNUSED(camera);
}

void PointCloudRenderer::render(QRhiCommandBuffer* cb, const TrackballCamera& camera, const PoseGizmo& gizmo,
                                QRhiRenderTarget* rt) {
  auto& res = *rhi_resources_;
  if (!rt) {
    return;
  }
  pollPickReadback();
  ensurePipeline(rt);
  ensureGizmoPipeline(rt);
  ensureShapePipeline(rt);
  uploadSphereMarkers();
  uploadGizmo(gizmo);

  const QSize fb = rt->pixelSize();
  const int fb_w = fb.width() > 0 ? fb.width() : std::max(1, width_);
  const int fb_h = fb.height() > 0 ? fb.height() : std::max(1, height_);

  UniformBlock ub{};
  Eigen::Matrix4f clip_corr = Eigen::Matrix4f::Identity();
  if (rhi_) {
    const QMatrix4x4 qt_corr = rhi_->clipSpaceCorrMatrix();
    std::memcpy(clip_corr.data(), qt_corr.constData(), sizeof(float) * 16);
  }
  const Eigen::Matrix4f mvp = clip_corr * camera.projectionMatrix() * camera.viewMatrix();
  std::memcpy(ub.mvp, mvp.data(), sizeof(ub.mvp));
  ub.pointSize = camera.pointSize();
  ub.blendWeight = blend_weight_;
  ub.scalarMin = scalar_min_;
  ub.scalarMax = scalar_max_;

  if (res.uniform_buffer) {
    void* ptr = res.uniform_buffer->beginFullDynamicBufferUpdateForCurrentFrame();
    if (ptr) {
      std::memcpy(ptr, &ub, sizeof(ub));
      res.uniform_buffer->endFullDynamicBufferUpdateForCurrentFrame();
    }
  }

  cb->beginPass(rt, QColor::fromRgbF(0.08f, 0.09f, 0.11f), {1.f, 0}, nullptr);
  if (pipeline_ready_ && res.pipeline && point_count_ > 0) {
    cb->setGraphicsPipeline(res.pipeline.get());
    cb->setViewport({0, 0, static_cast<float>(fb_w), static_cast<float>(fb_h)});
    cb->setShaderResources();
    const QRhiCommandBuffer::VertexInput vbuf(res.vertex_buffer.get(), 0);
    cb->setVertexInput(0, 1, &vbuf);
    cb->draw(point_count_);
  }
  if (res.gizmo_pipeline && gizmo_vertex_count_ > 0 && gizmo.enabled()) {
    GizmoUniform gu{};
    std::memcpy(gu.mvp, mvp.data(), sizeof(gu.mvp));
    void* gptr = res.gizmo_uniform_buffer->beginFullDynamicBufferUpdateForCurrentFrame();
    if (gptr) {
      std::memcpy(gptr, &gu, sizeof(gu));
      res.gizmo_uniform_buffer->endFullDynamicBufferUpdateForCurrentFrame();
    }
    cb->setGraphicsPipeline(res.gizmo_pipeline.get());
    cb->setShaderResources();
    const QRhiCommandBuffer::VertexInput gvbuf(res.gizmo_vertex_buffer.get(), 0);
    cb->setVertexInput(0, 1, &gvbuf);
    cb->draw(gizmo_vertex_count_);
  }
  if (res.shape_pipeline && shape_vertex_count_ > 0) {
    GizmoUniform su{};
    std::memcpy(su.mvp, mvp.data(), sizeof(su.mvp));
    void* sptr = res.shape_uniform_buffer->beginFullDynamicBufferUpdateForCurrentFrame();
    if (sptr) {
      std::memcpy(sptr, &su, sizeof(su));
      res.shape_uniform_buffer->endFullDynamicBufferUpdateForCurrentFrame();
    }
    cb->setGraphicsPipeline(res.shape_pipeline.get());
    cb->setShaderResources(res.shape_srb.get());
    const QRhiCommandBuffer::VertexInput svbuf(res.shape_vertex_buffer.get(), 0);
    cb->setVertexInput(0, 1, &svbuf);
    cb->draw(shape_vertex_count_);
  }
  cb->endPass();

  schedulePickPassIfIdle();
  if (pick_pass_scheduled_ && !res.pick_readback.in_flight && active_request_) {
    ensurePickTarget(fb_w, fb_h);
    renderPickPass(cb, camera, fb_w, fb_h, *active_request_);
    pick_pass_scheduled_ = false;
  }
  pollPickReadback();
}

}  // namespace psiviz
