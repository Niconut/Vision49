/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;

import java.util.Objects;


/**
 * A trapezoid-shaped velocity profile.
 *
 * <p>While this class can be used for a profiled movement from start to finish,
 * the intended usage is to filter a reference's dynamics based on trapezoidal
 * velocity constraints. To compute the reference obeying this constraint, do
 * the following.
 *
 * <p>Initialization:
 * <pre><code>
 * TrapezoidProfile.Constraints constraints =
 *   new TrapezoidProfile.Constraints(kMaxV, kMaxA);
 * TrapezoidProfile.State previousProfiledReference =
 *   new TrapezoidProfile.State(initialReference, 0.0);
 * </code></pre>
 *
 * <p>Run on update:
 * <pre><code>
 * TrapezoidProfile profile =
 *   new TrapezoidProfile(constraints, unprofiledReference, previousProfiledReference);
 * previousProfiledReference = profile.calculate(timeSincePreviousUpdate);
 * </code></pre>
 *
 * <p>where `unprofiledReference` is free to change between calls. Note that when
 * the unprofiled reference is within the constraints, `calculate()` returns the
 * unprofiled reference unchanged.
 *
 * <p>Otherwise, a timer can be started to provide monotonic values for
 * `calculate()` and to determine when the profile has completed via
 * `isFinished()`.
 */
// 1. Update the Constructor to ignore acceleration


/**
 * A custom Trapezoid Profile for FTC.
 * Features: Instant Ramp-Up (Zero Acceleration Phase) and Smooth Ramp-Down.
 * Compatible with Java Level 7 and above.
 */
public class myTrapezoidProfile {
    private int m_direction;
    private Constraints m_constraints;
    private State m_initial;
    private State m_goal;

    private double m_endFullSpeed;
    private double m_endDeccel;

    public static class Constraints extends TrapezoidProfile.Constraints {
        public double maxVelocity;
        public double maxDeceleration; // The smoothness of the stop

        public Constraints(double maxVelocity, double maxDeceleration) {
            this.maxVelocity = maxVelocity;
            this.maxDeceleration = maxDeceleration;
        }
    }

    public static class State {
        public double position;
        public double velocity;

        public State() {}

        public State(double position, double velocity) {
            this.position = position;
            this.velocity = velocity;
        }

        @Override
        public boolean equals(Object other) {
            if (other instanceof State) {
                State rhs = (State) other;
                return this.position == rhs.position && this.velocity == rhs.velocity;
            }
            return false;
        }

        @Override
        public int hashCode() {
            // Java 7 compatible hash
            int result;
            long temp;
            temp = Double.doubleToLongBits(position);
            result = (int) (temp ^ (temp >>> 32));
            temp = Double.doubleToLongBits(velocity);
            result = 31 * result + (int) (temp ^ (temp >>> 32));
            return result;
        }
    }

    public myTrapezoidProfile(Constraints constraints, State goal, State initial) {
        // Determine if we need to flip directions
        m_direction = (initial.position > goal.position) ? -1 : 1;

        m_constraints = constraints;
        m_initial = direct(initial);
        m_goal = direct(goal);

        // Distance to decelerate from max velocity to goal velocity (usually 0)
        double decelTime = m_constraints.maxVelocity / m_constraints.maxDeceleration;
        double distDecel = 0.5 * m_constraints.maxDeceleration * decelTime * decelTime;

        double totalDist = m_goal.position - m_initial.position;

        if (totalDist > distDecel) {
            // CASE 1: Long move. Instant jump to max velocity, then brake.
            double fullSpeedDist = totalDist - distDecel;
            m_endFullSpeed = fullSpeedDist / m_constraints.maxVelocity;
            m_endDeccel = m_endFullSpeed + decelTime;
        } else {
            // CASE 2: Short move. We can't hit max velocity without overshooting.
            // We calculate a lower "instant" velocity that allows a perfect landing.
            m_endFullSpeed = 0;
            // Time = sqrt(2 * distance / acceleration)
            m_endDeccel = Math.sqrt(2.0 * totalDist / m_constraints.maxDeceleration);

            // Adjust max velocity for this short trip so calculate() stays accurate
            m_constraints.maxVelocity = m_endDeccel * m_constraints.maxDeceleration;
        }
    }

    public State calculate(double t) {
        double pos;
        double vel;

        if (t < m_endFullSpeed) {
            // PHASE 1: INSTANT START (Full Speed)
            vel = m_constraints.maxVelocity;
            pos = m_initial.position + (m_constraints.maxVelocity * t);
        } else if (t <= m_endDeccel) {
            // PHASE 2: SMOOTH RAMP DOWN
            double timeLeft = m_endDeccel - t;
            vel = m_goal.velocity + (timeLeft * m_constraints.maxDeceleration);
            pos = m_goal.position - (m_goal.velocity + timeLeft * m_constraints.maxDeceleration / 2.0) * timeLeft;
        } else {
            // PHASE 3: AT GOAL
            vel = m_goal.velocity;
            pos = m_goal.position;
        }

        return direct(new State(pos, vel));
    }

    private State direct(State in) {
        return new State(in.position * m_direction, in.velocity * m_direction);
    }

    public double totalTime() {
        return m_endDeccel;
    }
}
