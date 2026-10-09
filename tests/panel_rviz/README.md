# Native panel acceptance

These tests publish synthetic ROS messages and open subscription-only Qt/RViz
windows. They do not start drivers, a recorder, a wheel interface or a camera.
Only the root agent schedules target builds and GUI execution.

## Qt save decision

Run `ctest --output-on-failure -R 'map_save_choice_ui|panel_save_flow'` in the
target `wc_bringup` build directory. Both tests use `QT_QPA_PLATFORM=offscreen`.

Required behaviors include explicit discard, closed question remaining pending,
affirmative selection of a new output directory, a visible folder browser,
one browser after hide/show, path cancellation returning to the decision, and
repeated cancellation followed by save. No test decision creates or deletes data.

The save executable returns one JSON object on stdout:

- `{"decision":"save","destination":"/absolute/new/directory"}`
- `{"decision":"discard","reason":"USER_DECLINED_SAVE"}`
- `{"decision":"pending","reason":"SAVE_DIALOG_CLOSED"}`

Actual stopping, saving, discarding and preservation of independent recordings
remain responsibilities of `mapping_app` and its controller.

## Three-column RViz

With target ROS and the candidate install sourced, run under the verified owned
desktop display:

```sh
python3 tests/panel_rviz/run_synthetic.py \
  --project-root /home/nvidia/wheelchair \
  --install-root /path/to/candidate/install \
  --output-root /configured/data/root/reports/new_panel_test
```

The test uses domain 217 with localhost-only DDS and requires a new output
directory. It checks actual message reception, all relevant RViz display status
levels, render frame counts, three visible columns, camera order and nonempty
screenshots. Review both PNGs for visible point clouds and maps; frame counts and
successful screenshots alone do not prove visually correct rendering.

The 2D viewport fits the first valid map once and offers an explicit “适配地图”
button to fit subsequent bounds. A new map does not overwrite manual pan/zoom.
Four corners include the occupancy-grid origin rotation and the current transform
to Fixed Frame; the helper matches MapDisplay's `Use Timestamp=false` rule
(now, with latest-transform fallback). `grid_fit` reports the applied center,
scale and fit count. The synthetic map is 8 by 5 metres and must initially fit
once at (4, 2.5), with its complete outline visible in the screenshot.

`WC_PANEL_SCREENSHOT`, `WC_PANEL_STATE_JSON` and `WC_PANEL_TEST_EXIT_MS` are
explicit observation controls. No file is overwritten. The timeout closes the
owned window, so these controls must not be applied to a real recording unless
the intended behavior is to end that recording.

The real recording command uses `unified_capture_rviz --session-root ...
--session-id ... --left-config ... --right-config ... --lidars both|left|right`.
`--read-only` excludes manual-control attachment and is used by synthetic tests.
`mapping_rviz` uses the normal RViz arguments plus `WC_PANEL_LAYOUT=unified`.

## Initialization regressions

Qt 5.15 creates the internal `QInputDialog` layout only when shown. Adding the
browse button before `exec()` dereferenced null. The fixed subclass adds it after
the base `showEvent`, then explicitly shows it and activates the layout because
the original child-show pass has already run.

Each independent RViz viewport must initialize its `RenderWindow` before creating
`VisualizationManager`. Otherwise the display-group enable step dereferences an
uninitialized Ogre scene. This order follows
[Humble VisualizationFrame::initialize](https://github.com/ros2/rviz/blob/humble/rviz_common/src/rviz_common/visualization_frame.cpp).

The whole Qt widget tree must also be attached and shown before that initialization.
Qt 5 initially embeds a QWindow beneath `ContainerFakeParent`, and changes to the
real native parent in the container's Show handler. See the
[Qt container implementation](https://github.com/qt/qtbase/blob/5.15/src/widgets/kernel/qwindowcontainer.cpp).
`RvizViewport` therefore separates construction from `initializeShown()`. The
caller shows the completed window, then initializes each independent viewport.
It verifies the native handles, visible container and nonzero dimensions, and
does not reinitialize a RenderWindow already initialized by an Expose event.
The `native_window` observation records initial/current Qt and Ogre window IDs
and dimensions. Stable IDs and healthy display status still require visible
point clouds/maps in the owned-window screenshots. Target native build 08 and
synthetic run 05 showed both capture point clouds and the mapping 3D/2D views;
earlier synthetic state checks passed while the custom viewport images were
black and GLX drawable creation failed. The runner now also rejects drawable
creation failures and invalid-framebuffer errors from the renderer log.

Humble also registers the same eight `RVIZ/` color materials in every manager
constructor. On target Ogre 1.12.1, declining a collision returns a null newly
created material; RViz dereferences it in `setAmbient`. A collision callback is
therefore not a usable adapter, despite the documented registration semantics.

`RvizColorMaterialRetention` holds the eight existing `MaterialPtr` instances,
removes only their exact registration handles, then lets RViz create the default
materials normally. The target `ResourceManager::remove` contract permits
unloading while keeping externally referenced objects alive; any previously
loaded material is explicitly reloaded if needed before the event loop resumes.
Post-construction checks require all eight new registrations and preservation of
the old loaded states. Retained references live with the viewport and are released
after its manager and render panel, before Ogre shutdown. This affects neither
the shared resource group nor unrelated materials, and installs no global listener.
The synthetic test checks eight retained materials in the second viewport and
distinct scene managers, render panels and view managers. Both views must continue
to process data and render correctly. Target compilation and rendered-image
acceptance remain required; this adapter does not address graphics driver or
drawable failures by itself.

## Repeated real-render lifecycle

The separate `test_panel_rviz_lifecycle` target uses a real owned desktop display,
not the Qt offscreen plugin. It is intentionally not an automatic headless ctest.

```sh
WC_PANEL_LIFECYCLE_OUTPUT=/configured/data/root/reports/new_lifecycle_run \
  /path/to/candidate/build/wc_bringup/test_panel_rviz_lifecycle
```

The output directory must not exist and its parent must already exist. Six
cycles render a red cube with the first viewport's actual `RVIZ/Red` material
before creating the second viewport. Owned screenshots must show both red cubes,
then the surviving cube after alternating destruction order. Weak references to
retired materials must expire after both views close and resource counts must
remain stable. A second test publishes synthetic blue/green clouds and verifies
independent rotation, pan, zoom, window resizing, and actual pixel changes when
the realtime/accumulated cloud switches are toggled.
A third case uses a translated, 90-degree-rotated map, verifies its fitted
bounds, then updates that map while preserving a manual view. Clicking the fit
button must restore the new center and a scale that leaves the declared margin.

Humble `RenderWindowImpl` creates a raw, private SceneManager for each window but
its destructor only destroys the RenderTarget. `RvizViewport` therefore destroys
its exclusive SceneManager after its VisualizationManager and RenderPanel, and
before releasing retained materials. It never assigns a shared scene through
`RenderPanel::initialize(..., true)`. See the
[Humble scene allocation and destructor](https://github.com/ros2/rviz/blob/humble/rviz_rendering/src/rviz_rendering/ogre_render_window_impl.cpp).
Installed-version ownership confirmation and native lifecycle execution remain
required; source preparation is not a passing native test.

Target lifecycle run 01 exposed four additional material registrations per
two-view cycle, while retired-default-material weak references already expired.
The diagnostic run identified `SelectionRect2`, `Shape5Material`, `SelectionRect3`
and `Shape7Material`, and both ViewManager QPointers remained alive after closing.
These are two separate Humble ownership gaps: the raw ViewManager is not deleted
by VisualizationManager, and SelectionManager deletes its rectangle without
unregistering the rectangle material/texture. The wrapper now clears the view
controller property tree while the context is alive, releases a surviving
ViewManager with a QPointer guard, and retires only the private selection resource
identities created during that manager's initialization. No unrelated resources
or shared groups are cleared. The test retains its exact resource-count assertion
and also requires both ViewManager QPointers to clear.

The next target run exposed a separate exit-order fault with PointCloud2 loaded:
`CallbackGroup::~CallbackGroup` dereferenced a control-block vtable after
`rviz_default_plugins` had already unloaded. The node keeps weak subscription
control blocks after displays are deleted. The wrapper now destroys that node
after VisualizationManager, but before releasing ViewManager's remaining plugin
factory. This is bounded per-viewport ownership, not a leaked factory or a
process-wide library pin. The interaction case also requires both ROS node weak
references to expire. Target native build 11 / lifecycle 11 passed all three
cases (5 QtTest results), including six stable-resource cycles and cloud exit.
Evidence: `rviz_crash_gdb_12.log`, `rviz_lifecycle_11.log` beneath the target's
`reports/panel_implementation_20261007` directory.

The production capture window additionally keeps a status subscription in a
sibling widget. `SubscriptionBase` retains NodeBase even after the Node itself
is released, so Qt's sibling destruction order cannot be used as the ownership
contract (`capture_crash_gdb_13.log`). All extra subscribers borrowed from a
viewport register `beforeNodeRelease` cleanup with their QObject context. The
viewport stops updates, releases those subscriptions, then tears down manager,
node and plugin factory in that order. Map fitting uses the same cleanup hook.
The cloud lifecycle test deliberately keeps a sibling status subscription alive
past both viewports and verifies that cleanup releases it.

Initial camera/status/dock layout updates can shrink the 2D viewport after the
first map message. Its one-time auto fit now waits for 500 ms without native
resize events. Once fitted, later map messages and resizes preserve the user's
view; the fit button explicitly applies the latest bounds. The synthetic runner
requires both map spans, including the 10% margin, to fit the final native pixel
dimensions. The lifecycle case includes two delayed initial resizes.

Final target native build 13 passed. Lifecycle 13 passed all 5 QtTest results;
synthetic 13 passed both production windows' subscriptions, renderer status,
geometry and normal exit checks; both save-dialog ctests passed. Manual review
of `rviz_synthetic_13/capture.png` and `mapping.png` confirmed both real cloud
views, four synthetic camera images, central clouds and the complete 2D map
boundary. These are synthetic software checks, not live sensor acceptance.

Build 14 adds a native-frame selector only for uncalibrated dual-lidar preview.
The fourth lifecycle case publishes blue `lidar_left` and green `lidar_right`
clouds without TF, uses actual combo-box keyboard events to select left/right/
left, and requires the matching cloud color alone to render. Calibrated preview,
mapping and single-lidar configurations must not expose this selector. Target
lifecycle 14 passed all 6 QtTest results; production synthetic 14 and both
save-dialog ctests also passed. Native build 14 is the frozen final candidate.

RViz status list names change to `Status: Ok/Error`; observation matches the
`StatusList` type, then checks `StatusProperty::getLevel()`, rather than matching
the literal label `Status`.
