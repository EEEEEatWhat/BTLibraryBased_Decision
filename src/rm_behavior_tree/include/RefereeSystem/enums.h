
#pragma once
#include <cstdint>

namespace RM_referee {
    enum class PacketType : uint16_t {
        GameStatus = 0x0001,
        GameResultEvent = 0x0002,
        GameRobotHP = 0x0003,

        PlaygroundEvent = 0x0101,
        ExtSupplyProjectileAction = 0x0102,
        RefereeWarningEvent = 0x0104,
        DartInfo = 0x0105,

        RobotState = 0x0201,
        PowerHeatData = 0x0202,
        RobotPosition = 0x0203,
        RobotBuff = 0x0204,
        AirSupportData = 0x0205,
        DamageEvent = 0x0206,
        ShootEvent = 0x0207,
        ProjectileAllowance = 0x0208,
        RobotRfidState = 0x0209,
        DartClientCmd = 0x020A,
        GroundRobotPosition = 0x020B,
        RadarMarkData = 0x020C,
        SentryInfo = 0x020D,
        RadarInfo = 0x020E,

        InterRobotCommsMessage = 0x0301,
        CustomRobotData = 0x0302,
        MinimapInteractionCommsMessage = 0x0303, 
        KeyboardMouseMessage = 0x0304,
        ClientMinimapRecv = 0x0305, 
        CustomClientData = 0x0306,
        MapData = 0x0307,
        CustomInfo = 0x0308
    };

    enum class GameType : uint8_t {
        Standard = 1,
        RMUTechnicalChallenge,
        ICRA_RMUA,
        RMUL_3V3,
        RMUL_1V1
    };

    enum class GameStage : uint8_t {
        NotStarted = 0,
        Preparing,
        SelfTest,
        Countdown,
        Started,
        EndScreen,
    };

    enum class GameResult : uint8_t {
        Tie = 0,
        RedWin,
        BlueWin,
    };

    enum class RobotID : uint8_t {
        Red1 = 1,
        Red2 = 2,
        Red3 = 3,
        Red4 = 4,
        Red5 = 5,
        RedAerial = 6,
        RedSentry = 7,
        RedDart = 8,
        RedRadar = 9,

        Blue1 = 101,
        Blue2 = 102,
        Blue3 = 103,
        Blue4 = 104,
        Blue5 = 105,
        BlueAerial = 106,
        BlueSentry = 107,
        BlueDart = 108,
        BlueRadar = 109,
    };

    enum class GraphicsType : uint8_t {
        Line = 0,
        Rectangle,
        Circle,
        Ellipse,
        FpNumber,
        IntNumber,
        String,
    };

    enum class GraphicsColor : uint8_t {
        TeamColor = 0,
        Yellow,
        Green,
        Orange,
        Violet,
        Pink,
        Cyan,
        Black,
        White,
    };
};
