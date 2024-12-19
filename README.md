# curobo_rviz

`ros2 launch curobo_rviz curobo_rviz.launch.py`

## RvizArgsPanel
### Current state
Used to change trajectory calculation parameters.

Retrieves the parameters at launch. Some can be updated on change, others must be updated with the "Confirm Changes" button.
### Future development
- [ ] Add multithreading to disabled the "Confirm Changes" button when clicked and process is loading
- [ ] Save and load the system's state

## AddObjectsPanel
### Current state
Used to manage objects in the scene.

The scene node part of the code must be revisited as it doesn't display the objects modelized by Rviz. Some objects from curobo aren't available with the RViz rendering class `Shape` and vice-versa. Color doesn't seem to work.
### Future development
- [ ] Disable boxes when parameter is not needed
- [ ] Refactor the code so Display is responsible for calling the services that adds and removes objects
- [ ] Persist object display after closing Rviz
- [ ] Persist object display that were added before opening Rviz
- [ ] Save and load the system's state
- [ ] Previsualize the object in the scene before adding it. Move it around with markers to position it intuitively.
- [ ] When object is selected, show parameters in boxes
- [ ] Merge all panels into one panel. Navigate with tabs.
- [ ] Select the path/to/mesh with a file explorer
