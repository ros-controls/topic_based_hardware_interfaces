# controller_manager_topic_hardware_component

```
ros2 bag play <bag_file> \
  --topics /controller_manager/introspection_data/names /controller_manager/introspection_data/values \
  --remap /controller_manager/introspection_data/names:=/epsiloncranebag/names\
          /controller_manager/introspection_data/values:=/epsiloncranebag/values
```