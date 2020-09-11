int32 width				# Total size in pixels
int32 height			# Total size in pixels
float32 resolution		# Map resolution
int32 pageSize			# Size for every page
int32 pageDim			# Dimension for every page
int32 numPages			# current number of allocated pages
# The origin of the map [m, m, rad].  This is the real-world pose of the
# cell (0,0) in the map.
geometry_msgs/Pose origin
