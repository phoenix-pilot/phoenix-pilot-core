from geopy import distance


class GeodeticDistance:
    def __init__(self, origin_latitude=0, origin_longitude=0) -> None:
        self.originLat = origin_latitude
        self.originLon = origin_longitude

    @staticmethod
    def distance(latitude1, longitude1, latitude2, longitude2):
        return distance.geodesic(
            (latitude1, longitude1),
            (latitude2, longitude2),
            ellipsoid='WGS-84'
        ).m

    def dist_from_origin(self, latitude, longitude):
        return self.distance(self.originLat, self.originLon, latitude, longitude)

    def latitude_diff(self, latitude):
        """Returns latitude difference in meters. Northward difference is positive and southward's negative"""

        res = self.dist_from_origin(latitude, self.originLon)

        if self.originLat > latitude:
            res = -res

        return res

    def longitude_diff(self, longitude):
        """Returns longitude difference in meters. Eastward difference is positive and westward's negative"""

        res = self.dist_from_origin(self.originLat, longitude)

        if self.originLon > longitude:
            res = -res

        return res
