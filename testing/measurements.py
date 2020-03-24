class Measurement:
    data = None
    R = None

""" All Measurements are either explicit or implicit """
class Implicit(Measurement):
    pass

class Explicit(Measurement):
    pass

""" Specific Measurement Enumerations """
class GPS_Explicit(Explicit):
    def __init__(self, x, R):
        self.data = x
        self.R = R

class LinRel_Explicit(Explicit):
    def __init__(self, delta, other_asset, R):
        self.data = delta
        self.R = R

class GPS_Implicit(Implicit):
    def __init__(self, R):
        self.R = R
    # data is None b/c it's implicit!

class LinRel_Implicit(Implicit):
    def __init__(self, other_asset, R):
        self.dst = other_asset
        self.R = R