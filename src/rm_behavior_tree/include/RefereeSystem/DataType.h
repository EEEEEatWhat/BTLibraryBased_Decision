/**
 * @author suzukisuncy
 * @date 2023/11/27
 * @brief 通过映射表来解析数据,映射表接受cmd_id，对应DJI裁判系统附录,基于DJI裁判系统协议V1.6
*/

#pragma once 
// #include <rclcpp/rclcpp.hpp>
#include <cstdint>
#include <cstddef>
#include <iostream>
#include <mutex>
#include "enums.h"
namespace RM_referee{
    // Packet header structure
    #pragma pack(1)
    struct PacketHeader {
        uint8_t SOF;
        uint16_t DataLength;
        uint8_t SequenceNumber;
        uint8_t CRC8;
    } ;
    #pragma pack()
    static_assert(sizeof(PacketHeader) == 5, "PacketHeader must be 5 bytes long with packing");
    static constexpr uint8_t StartOfFrame = 0xa5;

    // Base packet
    class RefereePacket {
        public:
            RefereePacket(){};
            virtual ~RefereePacket(){};
            virtual uint16_t GetID() = 0;
            virtual uint16_t GetDataLength() = 0;
            /**
             * @return 处理的字节数
            */
            virtual uint16_t SolvePacket(uint16_t cmd_id, uint8_t* data ,uint16_t data_size) {
                (void)cmd_id;
                (void)data;
                (void)data_size;
                std::cout<<"[Warning : This should not appear !]\n";
                return 0 ;
            };

    };
    
    #define GENERATEPACK(TYPE,STRUCT) \
    class TYPE##Packet : public RefereePacket { \
    protected:\
    public:\
        STRUCT m_value;\
        std::mutex m_mutex;\
        TYPE##Packet(){};\
        ~TYPE##Packet(){};\
        void testsuccess(){std::cout<<"\nSuccess!!!\n";};\
        static uint16_t StaticGetID(){return uint16_t(PacketType::TYPE);};\
        uint16_t GetID(){return StaticGetID();};\
        static uint16_t StaticGetDataLength(){return sizeof(STRUCT);};\
        uint16_t GetDataLength() {return StaticGetDataLength();};\
        virtual uint16_t SolvePacket(uint16_t cmd_id, uint8_t* data ,uint16_t data_size) override ; \
    };

    /**
    @brief  如何使用：
            How to use GENERATEPACK(TYPE,STRUCT):
            Example：0x0102 ExtSupplyProjectileActionPacket 4 ExtSupplyProjectileAction
            定义数据包结构体并断言数据包大小
            
            struct ExtSupplyProjectileActionStruct { 
                uint8_t reserved; 
                uint8_t supply_robot_id;  
                uint8_t supply_projectile_step; 
                uint8_t supply_projectile_num; 
            };
            static_assert(sizeof(ExtSupplyProjectileActionStruct) == 4, "ExtSupplyProjectileActionStruct must be 4 bytes long with packing");
            GENERATEPACK(ExtSupplyProjectileAction,ExtSupplyProjectileActionStruct)
    @warning  type,struct不要重复；type请查阅enum.h
    @brief 等效于：
            //0x0102 ExtSupplyProjectileActionPacket 4 ExtSupplyProjectileAction
            class ExtSupplyProjectileActionPacket : public RefereePacket { 
            TODO fix it
            };
    */


    //0x0001 GameStatusPacket 11 GameStatus
    #pragma pack(push, 1) 
    struct GameStatusStruct { 
        uint8_t game_type : 4; 
        uint8_t game_progress : 4; 
        uint16_t stage_remain_time; 
        uint64_t SyncTimeStamp;  }; 
    #pragma pack(pop)
    static_assert(sizeof(GameStatusStruct) == 11, "GameStatusStruct must be 11 bytes long with packing");
    GENERATEPACK(GameStatus,GameStatusStruct)

    //0x0002 GameResultEventPacket 1 GameResultEvent
    struct GameResultEventStruct { 
        uint8_t winner; }; 
    static_assert(sizeof(GameResultEventStruct) == 1, "GameResultEventStruct must be 1 bytes long with packing");
    GENERATEPACK(GameResultEvent,GameResultEventStruct)

    //0x0003 GameRobotHPPacket 32 GameRobotHP
    struct GameRobotHPStruct { 
        uint16_t red_1_robot_HP; 
        uint16_t red_2_robot_HP; 
        uint16_t red_3_robot_HP; 
        uint16_t red_4_robot_HP; 
        uint16_t red_5_robot_HP; 
        uint16_t red_7_robot_HP; 
        uint16_t red_outpost_HP; 
        uint16_t red_base_HP; 
        uint16_t blue_1_robot_HP; 
        uint16_t blue_2_robot_HP; 
        uint16_t blue_3_robot_HP; 
        uint16_t blue_4_robot_HP; 
        uint16_t blue_5_robot_HP; 
        uint16_t blue_7_robot_HP; 
        uint16_t blue_outpost_HP; 
        uint16_t blue_base_HP; };
    static_assert(sizeof(GameRobotHPStruct) == 32, "GameRobotHPStruct must be 32 bytes long with packing");
    GENERATEPACK(GameRobotHP,GameRobotHPStruct)

    //0x0101  PlaygroundEventPacket 4 PlaygroundEvent
    struct PlaygroundEventStruct { 
        uint32_t event_data; };
    static_assert(sizeof(PlaygroundEventStruct) == 4, "PlaygroundEventStruct must be 4 bytes long with packing");
    GENERATEPACK(PlaygroundEvent,PlaygroundEventStruct)

    //0x0102 ExtSupplyProjectileActionPacket 4 ExtSupplyProjectileAction
    struct ExtSupplyProjectileActionStruct { 
        uint8_t reserved; 
        uint8_t supply_robot_id;  
        uint8_t supply_projectile_step; 
        uint8_t supply_projectile_num; };
    static_assert(sizeof(ExtSupplyProjectileActionStruct) == 4, "ExtSupplyProjectileActionStruct must be 4 bytes long with packing");
    GENERATEPACK(ExtSupplyProjectileAction,ExtSupplyProjectileActionStruct)

    //0x0104 RefereeWarningEventPacket 3 RefereeWarningEvent
    #pragma pack(push, 1)
    struct  RefereeWarningEventStruct { 
        uint8_t level; 
        uint8_t offending_robot_id; 
        uint8_t count; };
    #pragma pack(pop)
    static_assert(sizeof(RefereeWarningEventStruct) == 3, "RefereeWarningEventStruct must be 3 bytes long with packing");
    GENERATEPACK(RefereeWarningEvent,RefereeWarningEventStruct)
    
    //0x0105 DartInfoPacket 2 DartInfo
    #pragma pack(push, 1)
    struct DartInfoPacketStruct { 
        uint8_t dart_remaining_time; 
        uint16_t dart_info; };
    #pragma pack(pop)
    static_assert(sizeof(DartInfoPacketStruct) == 3, "DartInfoPacketStruct must be 3 bytes long with packing");
    GENERATEPACK(DartInfo,DartInfoPacketStruct)

    //0x0201 RobotStatePacket 6 RobotState
    #pragma pack(push, 1)
    struct RobotStateStruct{ 
        uint8_t robot_id; 
        uint8_t robot_level; 
        uint16_t current_HP;   
        uint16_t maximum_HP; 
        uint16_t shooter_barrel_cooling_value; 
        uint16_t shooter_barrel_heat_limit; 
        uint16_t chassis_power_limit;   
        uint8_t power_management_gimbal_output : 1; 
        uint8_t power_management_chassis_output : 1;   
        uint8_t power_management_shooter_output : 1;  }; 
    #pragma pack(pop)
    static_assert(sizeof(RobotStateStruct) == 13 , "RobotStateStruct must be 3 bytes long with packing");
    GENERATEPACK(RobotState,RobotStateStruct)

    //0x0202 PowerHeatDataPacket 16 PowerHeatData
    struct PowerHeatDataStruct { 
        uint16_t chassis_voltage; 
        uint16_t chassis_current; 
        float chassis_power; 
        uint16_t buffer_energy; 
        uint16_t shooter_17mm_1_barrel_heat; 
        uint16_t shooter_17mm_2_barrel_heat; 
        uint16_t shooter_42mm_barrel_heat;}; 
    static_assert(sizeof(PowerHeatDataStruct) == 16, "PowerHeatDataStruct must be 16 bytes long with packing");
    GENERATEPACK(PowerHeatData,PowerHeatDataStruct)

    //0x0203 RobotPositionPacket 12 RobotPosition
    struct RobotPositionStruct { 
        float x; 
        float y; 
        float angle; }; 
    static_assert(sizeof(RobotPositionStruct) == 12, "RobotPositionStruct must be 16 bytes long with packing");
    GENERATEPACK(RobotPosition,RobotPositionStruct)

    //0x0204 RobotBuffPacket 5 RobotBuff
    #pragma pack(push, 1)
    struct RobotBuffStruct {     
        uint8_t recovery_buff;   
        uint8_t cooling_buff;   
        uint8_t defence_buff;   
        uint8_t vulnerability_buff; 
        uint16_t attack_buff; }; 
    #pragma pack(pop)
    static_assert(sizeof(RobotBuffStruct) == 6, "RobotBuffStruct must be 16 bytes long with packing");
    GENERATEPACK(RobotBuff,RobotBuffStruct)

    //0x0205 AirSupportDataPacket 2 AirSupportData
    struct AirSupportDataStruct { 
        uint8_t airforce_status; 
        uint8_t time_remain;};
    static_assert(sizeof(AirSupportDataStruct) == 2, "AirSupportDataStruct must be 16 bytes long with packing");
    GENERATEPACK(AirSupportData,AirSupportDataStruct)

    //0x0206 DamageEventPacket 1 DamageEvent
    #pragma pack(push, 1)
    struct DamageEventStruct { 
        uint8_t armor_id : 4; 
        uint8_t HP_deduction_reason : 4;};
    #pragma pack(pop)
    static_assert(sizeof(DamageEventStruct) == 1, "DamageEventStruct must be 16 bytes long with packing");
    GENERATEPACK(DamageEvent,DamageEventStruct)

    //0x0207 ShootEventPacket 7 ShootEvent
    #pragma pack(push, 1)
    struct ShootEventSruct { 
        uint8_t bullet_type;  
        uint8_t shooter_number; 
        uint8_t launching_frequency;  
        float initial_speed;  }; 
    #pragma pack(pop)
    static_assert(sizeof(ShootEventSruct) == 7, "ShootEventSruct must be 16 bytes long with packing");
    GENERATEPACK(ShootEvent,ShootEventSruct)

    //0x0208 ProjectileAllowancePacket 6 ProjectileAllowance
    struct ProjectileAllowanceStruct { 
        uint16_t projectile_allowance_17mm; 
        uint16_t projectile_allowance_42mm;   
        uint16_t remaining_gold_coin; }; 
    static_assert(sizeof(ProjectileAllowanceStruct) == 6, "ProjectileAllowanceStruct must be 16 bytes long with packing");
    GENERATEPACK(ProjectileAllowance,ProjectileAllowanceStruct)

    //0x0209 RobotRfidStatePacket 4 RobotRfidState
    struct RobotRfidStateStruct { 
        uint32_t rfid_status; };
    static_assert(sizeof(RobotRfidStateStruct) == 4, "RobotRfidStateStruct must be 16 bytes long with packing");
    GENERATEPACK(RobotRfidState,RobotRfidStateStruct)

    //0x020A DartClientCmdPacket 6 DartClientCmd
    struct DartClientCmdStruct { 
        uint8_t dart_launch_opening_status;   
        uint8_t reserved;   
        uint16_t target_change_time;   
        uint16_t latest_launch_cmd_time; };
    static_assert(sizeof(DartClientCmdStruct) == 6, "DartClientCmdStruct must be 16 bytes long with packing");
    GENERATEPACK(DartClientCmd,DartClientCmdStruct)

    //0x020B GroundRobotPositionPacket 40 GroundRobotPosition
    struct GroundRobotPositionStruct { 
        float hero_x;   
        float hero_y;   
        float engineer_x;   
        float engineer_y;   
        float standard_3_x;   
        float standard_3_y;   
        float standard_4_x;   
        float standard_4_y;   
        float standard_5_x;   
        float standard_5_y; };
    static_assert(sizeof(GroundRobotPositionStruct) == 40, "GroundRobotPositionStruct must be 16 bytes long with packing");
    GENERATEPACK(GroundRobotPosition,GroundRobotPositionStruct)

    //0x020C RadarMarkDataPacket 6 RadarMarkData
    struct RadarMarkDataStruct{ 
        uint8_t mark_hero_progress;   
        uint8_t mark_engineer_progress;   
        uint8_t mark_standard_3_progress;   
        uint8_t mark_standard_4_progress; 
        uint8_t mark_standard_5_progress; 
        uint8_t mark_sentry_progress; };
    static_assert(sizeof(RadarMarkDataStruct) == 6, "RadarMarkDataStruct must be 16 bytes long with packing");
    GENERATEPACK(RadarMarkData,RadarMarkDataStruct)

    //0x020D SentryInfoPacket 4 SentryInfo
    struct SentryInfoStruct { 
        uint32_t sentry_info; };
    static_assert(sizeof(SentryInfoStruct) == 4, "SentryInfoStruct must be 16 bytes long with packing");
    GENERATEPACK(SentryInfo,SentryInfoStruct)
    //0x020E RadarInfoPacket 1 RadarInfo

    //0x0301 InterRobotCommsMessagePacket 128 InterRobotCommsMessage


    //0x0302 CustomRobotDataPacket 30 CustomRobotData
    struct CustomRobotDataStruct { 
        uint8_t data[30]; };
    static_assert(sizeof(CustomRobotDataStruct) == 30, "CustomRobotDataStruct must be 30 bytes long with packing");
    GENERATEPACK(CustomRobotData,CustomRobotDataStruct)

    //0x0303 MinimapInteractionCommsMessagePacket 11 MinimapInteractionCommsMessage
    #pragma pack(push, 1)
    struct MinimapInteractionCommsMessageStruct { 
        float target_position_x; 
        float target_position_y; 
        uint8_t cmd_keyboard; 
        uint8_t target_robot_id; 
        uint8_t cmd_source; };
    #pragma pack(pop)
    static_assert(sizeof(MinimapInteractionCommsMessageStruct) == 11, "MinimapInteractionCommsMessageStruct must be 30 bytes long with packing");
    GENERATEPACK(MinimapInteractionCommsMessage,MinimapInteractionCommsMessageStruct)

    //0x0304 KeyboardMouseMessagePacket 12 KeyboardMouseMessage
    struct KeyboardMouseMessageStruct { 
        int16_t mouse_x; 
        int16_t mouse_y; 
        int16_t mouse_z; 
        int8_t left_button_down; 
        int8_t right_button_down; 
        uint16_t keyboard_value; 
        uint16_t reserved; };
    static_assert(sizeof(KeyboardMouseMessageStruct) == 12, "KeyboardMouseMessageStruct must be 30 bytes long with packing");
    GENERATEPACK(KeyboardMouseMessage,KeyboardMouseMessageStruct)

    //0x0305 ClientMinimapRecvPacket 10 ClientMinimapRecv
    #pragma pack(push, 1)
    struct ClientMinimapRecvStruct { 
        uint16_t target_robot_id; 
        float target_position_x; 
        float target_position_y; };
    #pragma pack(pop) 
    static_assert(sizeof(ClientMinimapRecvStruct) == 10, "ClientMinimapRecvStruct must be 30 bytes long with packing");
    GENERATEPACK(ClientMinimapRecv,ClientMinimapRecvStruct)

    //0x0306 CustomClientDataPacket 8 CustomClientData
    struct CustomClientDataStruct { 
        uint16_t key_value; 
        uint16_t x_position:12; 
        uint16_t mouse_left:4; 
        uint16_t y_position:12; 
        uint16_t mouse_right:4; 
        uint16_t reserved; }; 
    static_assert(sizeof(CustomClientDataStruct) == 8, "CustomClientDataStruct must be 30 bytes long with packing");
    GENERATEPACK(CustomClientData,CustomClientDataStruct)

    //0x0307 MapDataPacket 105 MapData
    #pragma pack(push, 1)
    struct MapDataStruct { 
        uint8_t intention; 
        uint16_t start_position_x; 
        uint16_t start_position_y; 
        int8_t delta_x[49]; 
        int8_t delta_y[49]; 
        uint16_t sender_id; };
    #pragma pack(pop)
    static_assert(sizeof(MapDataStruct) == 105, "MapDataStruct must be 30 bytes long with packing");
    GENERATEPACK(MapData,MapDataStruct)

    //0x0308 CustomInfoPacket 34 CustomInfo
    struct CustomInfoStruct{   
        uint16_t sender_id; 
        uint16_t receiver_id; 
        uint8_t user_data[30]; }; 
    static_assert(sizeof(CustomInfoStruct) == 34, "CustomInfoStruct must be 30 bytes long with packing");
    GENERATEPACK(CustomInfo,CustomInfoStruct) 

}