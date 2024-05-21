class Score():
    max = 675
    def calculateAccel(self, tmin, time):
        if (time > tmin * 1.5):
            return 4.5
        else:
            return 191 * ((1.5 * tmin / time) - 1) + 4.5        

    def calculateSkidpad(self, tmin, time):
        if (time > tmin * 1.5):
            return 3.5
        else:
            return 71.5/(1.5**2 - 1) * ((1.5 * tmin / time)**2 - 1) + 3.5

    def calculateAutocross(self, tmin, time):
        if (time > tmin * 1.5):
            return 6.5
        else:
            return 237 * ((1.5 * tmin / time) - 1) + 6.5

    def calculateEndurance(self, tmin, time):
        if (time > tmin * 1.5):
            return 25
        else:
            return 500 * ((1.5 * tmin / time) - 1) + 25

    def calculateEfficiency(self, co2min, efMax, co2, tmin, time):
        efMin = (co2min / 1.45 * 60.06)
        ef = (tmin * co2min / time * co2)
        return 100 * ((efMin / ef) - 1) * ((efMin / efMax) - 1) 