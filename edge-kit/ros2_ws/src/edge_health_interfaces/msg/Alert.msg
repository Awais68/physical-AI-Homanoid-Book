# Alert message for Physical AI System Monitoring
# Represents a threshold violation or system alert

std_msgs/Header header
string alert_id
float64 timestamp
string severity
string component
string metric_name
float64 current_value
float64 threshold_value
string message
string node_name
string alert_type
