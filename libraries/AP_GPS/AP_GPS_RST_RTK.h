#pragma once

#include "AP_GPS.h"
#include "GPS_Backend.h"
class AP_GPS_RST_RTK : public AP_GPS_Backend
{
public:
    AP_GPS_RST_RTK(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port);

    ~ AP_GPS_RST_RTK();

    static bool _detect(struct RST_RTK_detect_state &state, uint8_t data);
    bool read() override;

    AP_GPS::GPS_Status highest_supported_status(void) override { return AP_GPS::GPS_OK_FIX_3D_RTK_FIXED; }

    const char *name() const override { return "rst-rtk"; }
private:

    // Buffer parse & GPS state update
    bool        _parse_gps();


    // Packet checksum accumulators
    uint16_t    _XOR_check;
    uint16_t    _Checksum;

    // State machine state
    uint8_t         _step;
    uint16_t        _payload_length;
    uint16_t        _payload_counter;


    uint32_t        _last_pos_time;
    uint32_t        _last_vel_time;

    // do we have new position information?
    bool            _new_position:1;
    bool            _new_speed:1;


    struct PACKED rst_rtk{
        	uint16_t	Version;		    /**格式版本*/
            uint16_t    Length;		        /**数据包总长度*/
            uint16_t    Freq;		        /**数据输出频率*/
            float		Time_utc;		    /**UTC时间*/
            uint16_t	Year_utc;		    /**UTC年份*/
            uint16_t	Month_tc;  		    /**UTC月份*/
            uint16_t	Day_uc;             /**UTC日*/
            uint16_t	Hour_uc;            /**UTC时*/
            uint16_t	Min_uc;             /**UTC分*/
            uint16_t	Sec_uc;             /**UTC秒（十毫秒）*/
            double      Latitude;           /**纬度（度）*/
            double      Longitude;          /**经度（度）*/
            double      Altitude;           /**海拔高（m）*/
            float       Eph;                /**水平误差（m）*/
            float       Epv;                /**垂直误差（m）*/
            float       Vel_earth;          /**GPS地速m/s*/
            float       Angle_TrackTrue;    /**地速方向*/
            float       Angle_Heading;      /**偏航角度值（0～359.999）*/
            float       Angle_Pitch;        /**俯仰角度值（-90～+90）*/
            double      Vel_n;              /**GPS北向速度m/s*/
            double      Vel_e;              /**GPS东向速度m/s*/
            double      Vel_u;              /**GPS天向速度，向上为正m/s*/
            uint16_t    Satellites_used;    /**使用卫星数*/
            uint16_t    Satellites_track;   /**可见卫星数*/
            float       vel_neu_valid;      /**北东天速度是否有效0：无效，16：单点位置，50：RTK固定解*/
            uint16_t    Fix_type;           /**GPS状态*/
            float       Head_state;         /**偏航角状态*/
            float       Head_deviation;     /**偏航角标准差*/
            uint16_t    INS_state;          /**是否启用惯导0：未使用惯导，1：使用惯导*/
            double      GNSS_Alt_delta;     /**大地水准面和所选椭球面坐标系之间的高度差（m）*/
            double      Ellipsoidal_H;      /**椭球高，测量点与椭球面的正交距离（m）*/
            uint16_t    Diff_age;           /**差分龄期*/
            uint16_t    Reserve;            /**预留位*/
        
    };                          /**使用中海达RTK*/

    union PACKED {
        DEFINE_BYTE_ARRAY_METHODS
        rst_rtk rtk;
    }_buffer;
    
};