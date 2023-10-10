from enum import IntEnum, Enum


class SpeedClass(Enum):
    Urban = 13.889
    Highway = 33.333


class RoadOptions(IntEnum):
    SWITCHLEFT = 0
    SWITCHRIGHT = 1
    FOLLOW = 2


class RoadOptionsExtended(IntEnum):
    SWITCHLEFT = 0
    SWITCHRIGHT = 1
    FOLLOW = 2
    LEFT = 3
    RIGHT = 4


class CarFollowingModel(Enum):
    Krauss = "Krauss"
    KraussOrig1 = "	KraussOrig1"
    PWagner2009 = "PWagner2009"
    BKerner = "BKerner"
    IDM = "IDM"
    IDMM = "IDMM"
    EIDM = "EIDM"
    KraussPS = "KraussPS"
    KraussAB = "KraussAB"
    SmartSK = "SmartSK"
    Wiedemann = "Wiedemann"
    W99 = "W99"
    Daniel1 = "Daniel1"
    ACC = "ACC"
    CACC = "CACC"


class DynamicsModel(Enum):
    KS = "KS"
    ST = "ST"
    STD = "STD"
    MB = "MB"
    God = "God"


class CarModel(Enum):
    FordEscort = "FordEscort"
    BMW320i = "BMW320i"
    VWVanagon = "VWVanagon"


class GeneratableRoad(Enum):
    Intersection = "Intersection"
    HighwayEntry = "HighwayEntry"
    HighwayExit = "HighwayExit"
    HighwayDrive = "HighwayDrive"
    Highway = "Highway"
    Roundabout = "Roundabout"
    CountryRoad = "CountryRoad"


class FeatureScaling(Enum):
    Normalize = "Normalize"
    Standardize = "Standardize"


class Observer(Enum):
    AvailableOptions = "AvailableOptions"
    BirdsEye = "BirdsEye"
    EgoVehicle = "EgoVehicle"
    RadiusVehicle = "RadiusVehicle"
    Waypoints = "Waypoints"
    SubGoals = "SubGoals"
    RoadShape = "RoadShape"
    Road = "Road"
    CarlaCamera = "CarlaCamera"


class ActionSpace(Enum):
    Continuous = "Continuous"
    Discretized = "Discretized"
    Semantic = "Semantic"


class CarlaSensor(Enum):
    RGBCamera = "RGBCamera"
    DepthCamera = "DepthCamera"
