ADD_LIBRARY(hbridge2 SHARED Protocol.cpp Controller.cpp Reader.cpp Writer.cpp Encoder.cpp)
TARGET_LINK_LIBRARIES(hbridge2 ${CANBUS_DRIVER_LIBRARIES} ${BASE_LIBRARIES})

