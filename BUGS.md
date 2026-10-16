# Known bugs

## Concurrency

- When fusion is running, a new dataset can be added and becomes active
automatically. Then the fusion process finishes and the 3d scene updates to the
fused cloud, even though the corresponding datset is no longer active

# Improvements, to be openspecced

## UI

- The split between params and cloud view should be resizable, params should only use as much space as needed.
- Fly to point focus is good, but distance too far.
- Keyboard shortcuts: Esc for closing polygon or window, Ctrl-W, Cmd-Q

## Functionality

- Noise and outlier filtering on the fused cloud. Edges are noisy.