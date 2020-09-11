# Message contains annotation update
rns_msgs/Annotation[] annotations

int32 ActionAdd = 1 #;    /// Add new annotations. If they are already exist - they will be modified on server
int32 ActionSet = 2 #;    /// Override whole annotation list
int32 ActionDelete = 3 #; /// Delete annotations that stored in AnnotationsUpdate.annotations array. They are found by their IDs
int32 ActionNotify = 4 #  /// Notify that there are updates for the server

# takes one of aforementioned values
int32 action
# Current server revision
int64 revision
# List of updated identifiers. Actual meaning corresponds to action type
int64[] updated