std_msgs/Header header
unique_identifier_msgs/UUID id
tier4_simulation_msgs/DummyObjectInitialState initial_state
autoware_perception_msgs/ObjectClassification classification
autoware_perception_msgs/Shape shape
float32 max_velocity
float32 min_velocity

uint8 ADD=0
uint8 MODIFY=1
uint8 DELETE=2
uint8 DELETEALL=3
int32 action
