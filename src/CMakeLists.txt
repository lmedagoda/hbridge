ADD_LIBRARY(hbridge SHARED Protocol.cpp Controller.cpp Reader.cpp Writer.cpp Encoder.cpp CanBusInterface.cpp ../firmware/common/packets.c)
TARGET_LINK_LIBRARIES(hbridge ${CANBUS_DRIVER_LIBRARIES} ${BASE_LIBRARIES})

INSTALL(TARGETS hbridge
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib)
INSTALL(FILES 
    Controller.hpp
    Encoder.hpp
    Reader.hpp
    Writer.hpp
    Protocol.hpp
    CanBusInterface.hpp
    MotorDriverTypes.hpp
    DESTINATION include/hbridge)
INSTALL(FILES ../firmware/common/packets.h DESTINATION include/hbridge/firmware/common)
INSTALL(FILES ../firmware/common/pid.h DESTINATION include/hbridge/firmware/common)

