# Recording Status Message
# Information about data recording status

std_msgs/Header header

# Recording state
bool recording_active
string current_file
float64 duration_seconds

# Performance metrics
int32 queue_size
float64 data_rate_hz

# Storage info
float64 file_size_mb

# Success marker
bool marked_successful 