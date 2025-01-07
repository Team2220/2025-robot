package frc.lib.units;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.derive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Unit;
import edu.wpi.first.units.measure.Velocity;

public final class UnitsUtil {
    private UnitsUtil() {
        // hi - dont make instances
    }

    /* Standard unit of measurement for frequency, hertz */
    public static final Frequency hertz = new Frequency(1, "Hertz", "Hz");
    /* Standard unit of measurement for frequency, mega hertz */
    public static final Frequency megaHertz = derive(hertz).aggregate(1000).named("Mega Hertz").symbol("Mhz")
            .make();
    /* Standard unit of measurement for frequency, giga hertz */
    public static final Frequency gigaHertz = derive(megaHertz).aggregate(1000).named("Giga Hertz").symbol("Ghz")
            .make();

    public static <U extends Unit<U>> U abs(U measure) {
        double dMeasure = measure.in(measure.unit());
        return measure.unit().of(Math.abs(dMeasure));
    }

    public static final Distance distanceForWheel(Distance wheelDiameter, Angle rotations) {
        var distance = Math.PI * wheelDiameter.in(Meters) * rotations.in(Rotations);
        return Meters.of(distance);
    }

    public static final LinearVelocity velocityForWheel(Distance wheelDiameter, AngularVelocity rotations) {
        var distance = Math.PI * wheelDiameter.in(Meters) * rotations.in(RotationsPerSecond);
        return MetersPerSecond.of(distance);
    }

    public static final Velocity<Velocity<Angle>> RotationsPerSecSquared = RotationsPerSecond.per(Seconds);

    public static final Velocity<Velocity<Velocity<Angle>>> RotationsPerSecCubed = RotationsPerSecSquared.per(Seconds);
    public static <U extends Unit<U>> U inputModulus(U value, U min, U max){
        U unit = value.unit();
        double dvalue = value.in(unit);
        double dmin = value.in(unit);
        double dmax = value.in(unit);
        return unit.of(MathUtil.inputModulus(dvalue, dmin, dmax)); 
    }

    public static final Angle angleModulus(Angle value){

      return Radian.of(MathUtil.angleModulus(value.in(Radian)));

    };
}
