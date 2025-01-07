package frc.lib.tunables;

import java.util.function.Consumer;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Unit;
import frc.lib.functional.BooleanObjectConsumer;

public class TunableMeasure<U extends Unit<U>> {
    private TunableDouble tunable;
    private U unit;

    public TunableMeasure(String name, U defaultValue, String tab) {
        unit = defaultValue.unit();
        tunable = new TunableDouble(name, defaultValue.in(unit), tab);
    }

    public TunableMeasure(String name, U defaultValue, String tab, Consumer<U> onChange) {
        this(name, defaultValue, tab);
        addChangeListener(onChange);
    }

    public TunableMeasure(String name, U defaultValue, String tab, BooleanObjectConsumer<U> onChange) {
        this(name, defaultValue, tab);
        addChangeListener(onChange);
    }

    public U getValue() {
        return unit.of(tunable.getValue());
    }

    public void addChangeListener(BooleanObjectConsumer<U> onChange) {
        tunable.addChangeListener((isInit, value) -> {
            onChange.accept(isInit, unit.of(value));
        });
    }
        public void addChangeListener(Consumer<U> onChange) {
            addChangeListener((isInit, value)->onChange.accept(value));
        }
}
