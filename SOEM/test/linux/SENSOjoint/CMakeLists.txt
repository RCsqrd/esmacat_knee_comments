
set(SOURCES SENSOjoint.c)
add_executable(SENSOjoint ${SOURCES})
target_link_libraries(SENSOjoint soem)
install(TARGETS SENSOjoint DESTINATION bin)
