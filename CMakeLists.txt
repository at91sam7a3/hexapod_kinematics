project(hexapod)

aux_source_directory(src HEXAPOD_SRC_LIST)

add_library(hexapod STATIC ${HEXAPOD_SRC_LIST})

