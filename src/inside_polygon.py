from location import LocationObject

class Geospacial:
    lVertices = [
        LocationObject(-28.1567392, 153.3280381),
        LocationObject(-28.1568149, 153.3281870),
        LocationObject(-28.1567688, 153.3282420),
        LocationObject(-28.1567392, 153.3282997),
        LocationObject(-28.1567357, 153.3283386),
        LocationObject(-28.1567365, 153.3284806),
        LocationObject(-28.1567424, 153.3285516),
        LocationObject(-28.1566336, 153.3285744),
        LocationObject(-28.1565603, 153.3285852),
        LocationObject(-28.1565130, 153.3285074),
        LocationObject(-28.1565106, 153.3284323),
        LocationObject(-28.1565036, 153.3283491),
        LocationObject(-28.1564882, 153.3282392),
        LocationObject(-28.1565213, 153.3281895),
        LocationObject(-28.1565650, 153.3281185),
        LocationObject(-28.1566253, 153.3280809),
        LocationObject(-28.1567392, 153.3280381)
    ]

    def determineIfPointLiesWithinPolygon(self, locationToTest):
            # Check that the argument is actually a LocationObject.
            if isinstance(locationToTest, LocationObject):
                # Set crossings value to initial zero value.
                iCrossings = 0

                # For each segment of the polygon (i.e. line drawn between each of the polygon's vertices) check whether
                # a ray cast straight down from the test location intersects the segment (keep in mind that the segment
                # may be intersected either coming or going). If the ray does cross the segment, increment the iCrossings
                #  variable.
                for iVertex in range(0, len(self.lVertices)):

                    fSegmentStartX = self.lVertices[iVertex].fLongitude
                    fSegmentStartY = self.lVertices[iVertex].fLatitude

                    # If the last vertex is being tested then it joins the vertex at index 0.
                    if iVertex < len(self.lVertices) - 1:
                        fSegmentEndX = self.lVertices[iVertex + 1].fLongitude
                        fSegmentEndY = self.lVertices[iVertex + 1].fLatitude
                    else:
                        fSegmentEndX = self.lVertices[0].fLongitude
                        fSegmentEndY = self.lVertices[0].fLatitude

                    fTestPointX = locationToTest.fLongitude
                    fTestPointY = locationToTest.fLatitude

                    # Quickly check that the ray falls within the range of values of longitude for the start and end of
                    # the segment (keep in mind that the segment may be headed east or west).
                    if ((fSegmentStartX < fTestPointX) and (fTestPointX < fSegmentEndX)) or ((fSegmentStartX > fTestPointX) and (fTestPointX > fSegmentEndX)):
                        # Check if the segment is crossed in the Y axis as well (the point may lie below the segment).
                        fT = (fTestPointX - fSegmentEndX) / (fSegmentStartX - fSegmentEndX)
                        fCrossingY = ((fT * fSegmentStartY) + ((1 - fT) * fSegmentEndY))
                        if fCrossingY >= fTestPointY:
                            iCrossings += 1

                    # Check if the point lies on the segment, in particular if it lies precisely on a vertical line segment.
                    if (fSegmentStartX == fTestPointX) and (fSegmentStartY <= fTestPointY):
                        if fSegmentStartY == fTestPointY:
                            iCrossings += 1
                        if fSegmentEndX == fTestPointX:
                            if ((fSegmentStartY <= fTestPointY) and (fTestPointY <= fSegmentEndY)) or ((fSegmentStartY >= fTestPointY) and (fTestPointY >= fSegmentEndY)):
                                iCrossings += 1
                        elif fSegmentEndX > fTestPointX:
                            iCrossings += 1
                        if self.lVertices[iVertex - 1].fLongitude > fTestPointX:
                            iCrossings += 1

                # If the number of segment crossings is an even number the point lies outside the polygon. If there is an
                #  odd number of crossings the point lies inside.
                iRemainder = iCrossings % 2
                if iRemainder != 0:
                    return True  # Point is inside the polygon.
                else:
                    return False  # Point is outside the polygon.

            else:
                raise TypeError("An object of a class other than LocationObject passed to the "
                                "determineIfPointLiesWithinPolygon() function.")