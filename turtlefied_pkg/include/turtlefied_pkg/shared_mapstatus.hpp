#pragma once

class SharedMapStatus
{
public:
    static SharedMapStatus& getInstance()
    {
        static SharedMapStatus instance;
        return instance;
    }

    bool getMapStatus() const { return isMapSaved; }
    void setMapStatus(const bool& ismapsaved) { isMapSaved = ismapsaved; }

private:
    bool isMapSaved;
    SharedMapStatus() {}
};