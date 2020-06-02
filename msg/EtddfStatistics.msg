uint32 seq
time stamp
uint32 implicit_count
uint32 explicit_count
float32[] delta_tiers
# Number of times each delta_tier has been sent (corresponds with index in delta_tiers)
uint32[] buffer_counts
float32 current_lowest_multiplier
string[] current_lowest_buffer
bool overflown