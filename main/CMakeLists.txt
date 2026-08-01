list(APPEND ADD_INCLUDE "include")
append_srcs_dir(ADD_SRCS "src")
list(APPEND ADD_REQUIREMENTS basic vision nn peripheral network cpp-httplib)
register_component()
