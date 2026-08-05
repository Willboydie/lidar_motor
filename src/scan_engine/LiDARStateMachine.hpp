#pragma once

enum class ScanMode {
    COARSE,
    POINT2POINT,
    OFF,
};

enum class DynamicRoutine {
    NONE,
    SEEKING_NEUTRAL,
    WINDING_UP_TO_COARSE,
    TAPERING_DOWN_FROM_COARSE,
};


class LiDARStateMachine {
    public:
        LiDARStateMachine();
        ~LiDARStateMachine() = default;

        void on()

        // dynamic routine management
        void dynamicRoutineUpdate();

    private:
        void followSeekingNeutralRoutine();
        void followWindingUpToCoarseRoutine();
        void followTaperDownFromCoarseRoutine();

        // dynamic routine setters
        void initiateWindingUpToCoarse();
        void initiateTaperDownFromCoarse();
        void seekNeutral();

        ScanMode scanMode = ScanMode::OFF;
        DynamicRoutine routineBuffer[2] = {DynamicRoutine::NONE, DynamicRoutine::NONE};
        int routineBufferIndex = 0;
};