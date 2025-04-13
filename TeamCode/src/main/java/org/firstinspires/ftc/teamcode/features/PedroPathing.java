package org.firstinspires.ftc.teamcode.features;

import androidx.annotation.NonNull;

import dev.frozenmilk.dairy.core.Feature;
import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;

public final class PedroPathing implements Feature {

    private Dependency<?> dependency = new SingleAnnotation<>(PedroPathing.Attach.class);

    @NonNull
    @Override public Dependency<?> getDependency() { return dependency; }

    @Override public void setDependency(@NonNull Dependency<?> dependency) {
        this.dependency = dependency;
    }



    public @interface Attach {}
}
