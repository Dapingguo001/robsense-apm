#include "AP_GPS.h"
#include "AP_GPS_RST_RTK.h"

#include <AP_HAL/AP_HAL.h>
#include <math.h>

#define RST_RTK_DEBUGGING 0

#if RST_RTK_DEBUGGING
 # define Debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); hal.scheduler->delay(1); } while(0)
#else
 # define Debug(fmt, args ...)
#endif

extern const AP_HAL::HAL& hal;

AP_GPS_RST_RTK::AP_GPS_RST_RTK(AP_GPS &_gps, AP_GPS::GPS_State &_state,
                       AP_HAL::UARTDriver *_port) :
    AP_GPS_Backend(_gps, _state, _port),
    _XOR_check(0),
    _payload_length(134)
{

}

AP_GPS_RST_RTK::~AP_GPS_RST_RTK()
{

}

bool
AP_GPS_RST_RTK::_detect(struct RST_RTK_detect_state &state, uint8_t data)
{
reset:
	switch (state.step) {
        case 0:
            state.step = 0;
            if(data == 0xAA){
                state.step ++;
                state.XOR_check = (uint16_t)(state.XOR_check ^ data); 
            }

            break;

        case 1:
            if(data == 0x33){
                state.payload_counter = 0;
                state.step ++;
                state.XOR_check = (uint16_t)(state.XOR_check ^ data);
            }
            else{
                state.XOR_check = 0;
                state.step = 0;
            }

            break;      

        case 2: //payload
            state.XOR_check = (uint16_t)(state.XOR_check ^ data);
            if (++state.payload_counter == state.payload_length){
                state.step ++;
            }
            break;

        case 3:
            state.step ++;
            state.Checksum = (uint16_t)((uint16_t)(data) << 8);
            break;

        case 4:
            state.Checksum = (uint16_t)(state.Checksum + data);
            if((state.XOR_check << 8) != state.Checksum){
                Debug("rst rtk checksum err");
            }
            else{
                return true;
            }
            state.step = 0;
            state.XOR_check = 0;
			goto reset;
            break;
        
        default:
            break;
    }
    return false;
}

bool
AP_GPS_RST_RTK::read(void)
{
    int16_t numc;
    uint8_t data;
    bool parsed = false;
    numc = port->available();
    for (int16_t i = 0; i < numc; i++) {        // Process bytes received
        // read the next byte
        data = port->read();
        	
        reset:
        switch (_step) {
            case 0:
                _step = 0;
                if(data == 0xAA){
                    _step ++;
                    _XOR_check = (uint16_t)(_XOR_check ^ data);
                }

                break;

            case 1:
                if(data == 0x33){
                    _step ++;
                    _payload_counter = 0;
                    _XOR_check = (uint16_t)(_XOR_check ^ data);
                }
                else{
                    _step = 0;
                    _XOR_check = 0;
                }

                break;       

            case 2:
                _XOR_check = (uint16_t)(_XOR_check ^ data);

                if (_payload_counter < sizeof(_buffer)) {
                    _buffer[_payload_counter] = data;
                }
                if (++_payload_counter == _payload_length){
                    _step ++;
                }
                break;

            case 3:
                _step ++;
                _Checksum = (uint16_t)((uint16_t)(data) << 8);
                break;

            case 4:
                _Checksum = (uint16_t)(_Checksum + data);
                if((_XOR_check << 8) != _Checksum){
                    Debug("rst rtk checksum err");
                }
                else{
                    if (_parse_gps()) {
                        parsed = true;
                    }
                }
                _step = 0;
                _XOR_check = 0;
			    goto reset;
                break;

            default:
                break;
        }
    }     
    return parsed;
}

bool
AP_GPS_RST_RTK::_parse_gps()
{
    //缓冲区中获取数据
    _last_pos_time        = _buffer.rtk.Sec_uc;
    state.location.lng    = _buffer.rtk.Latitude * 1e7;
    state.location.lat    = _buffer.rtk.Longitude * 1e7;
    state.location.alt    = _buffer.rtk.Altitude * 1e2;

    if(_buffer.rtk.Fix_type  == 0){
        state.status = AP_GPS::NO_FIX;
    }
    else if(_buffer.rtk.Fix_type  == 1){ 
        state.status =  AP_GPS::GPS_OK_FIX_3D;     
    }
    else if (_buffer.rtk.Fix_type  == 2){  //DGPS
        state.status =  AP_GPS::GPS_OK_FIX_3D_DGPS;          
    }
    else if(_buffer.rtk.Fix_type  == 4){    //float rtk
         state.status =  AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT;
    }
    else if(_buffer.rtk.Fix_type  == 5){    //fix rtk
         state.status =  AP_GPS::GPS_OK_FIX_3D_RTK_FIXED;
    }
    else{
        state.status =  AP_GPS::NO_FIX;
    }
    _new_position = true;

    state.horizontal_accuracy = _buffer.rtk.Eph*1.0e-3f;
    state.vertical_accuracy = _buffer.rtk.Epv*1.0e-3f;
    state.have_horizontal_accuracy = true;
    state.have_vertical_accuracy = true;

    // SVs
    state.num_sats    = _buffer.rtk.Satellites_used;

    // velocity  
    _last_vel_time         = _buffer.rtk.Sec_uc;
    state.velocity.x = _buffer.rtk.Vel_n;
    state.velocity.y = _buffer.rtk.Vel_e;
    state.velocity.z = -_buffer.rtk.Vel_u;

    state.ground_speed     = _buffer.rtk.Vel_earth*0.001f;          // m/s
    state.ground_course    = (float)(atan2f(state.velocity.x * 0.01745329f, state.velocity.x * 0.01745329f) * 180.0f);   //运动航向（实际运动方向，不是机头航向       // Heading 2D deg * 100000
    state.have_vertical_velocity = true;
    state.have_speed_accuracy = true;
    state.speed_accuracy = 0.5f;   //速度精度估计，预测值
    _new_speed = true;

    state.hdop        = 0;
    state.vdop        = 0;

    state.last_gps_time_ms = AP_HAL::millis();

    // time
    state.time_week_ms    = _buffer.rtk.Sec_uc;
    state.time_week       = floor(_buffer.rtk.Day_uc/7);

    if (_new_position && _new_speed && _last_vel_time == _last_pos_time) {
        _new_speed = _new_position = false;
//        hal.console->printf("yuwenbin......gps...test......%d.\n",11);
        return true;
    }

    return false;
}