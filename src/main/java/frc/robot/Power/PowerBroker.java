package frc.robot.Power;

import static frc.robot.Power.PowerBank.centralBank;

import java.util.function.Supplier;

/**
 * This class represents a broker that negotiates power allowance at the power bank.
 * Each power consumer on the robot, i.e. every subsystem, should hold an object of this class.
 * The broker is instanciated with a priority supplier that is a {@code Supplier<Double>}.
 * For the broker may ask the consumer "what is your current priority?" anytime during the negotiation.
 * Then the consumer class must use the {@link #requestPower} method of that object before drawing any power.
 * 
 * Example of use:
 * <pre>{@code
 *  import static frc.robot.Power.PowerBroker;
 * 
 *  public class MySubsystem extends SubsystemBase {
 *      PowerBroker powerBroker = new PowerBroker(() -> 1.0); // Simple priority always = 1
 * 
 *      void motorOn() {
 *          double power = powerBroker.requestPower(42);
 *          double busPerc = myCalcFromWattsToPerc(power);
 *          motor.set(busPerc);
 *      }
 *  }
 * }</pre>
 */
public class PowerBroker {
    private PowerBank bank;
    private Supplier<Double> prioritySupplier;   // Higher number means higher priority
    private double powerRequested;               // in watts
    private double powerMinimum;
    private double powerAllowed;
    private Status status = Status.awaiting;

    public enum Status {
        awaiting,
        reserved,
        consuming,
        disconnected
    }

    public String toString() {
        return "(Req:" + powerRequested + " -> " + powerAllowed + ", " + status + ")";
    }

    /**
     * Constructs a broker that will open an account at the central power bank.
     * The consumer class must provide a "priority supplier" that can be as simple as returning
     * a constant or a variable, or as elaborate as calculating the priority value in flight.
     * The simplest supplier can be provided as <code>new PowerBroker(() -> 1.0)</code>
     * <P>
     * The range of the priorities is on the programmers' discretion.
     * Priority is relative among the consumers of the same power bank.
     * However, zero priority means no priority at all and should never be used to avoid
     * possible divide by zero situation in the power allocation logic.
     * The higher priority means more power the consumer will get in case of competition.
     * 
     * @param prioritySupplier  a <code>Double</code> supplier that will return the consumer's priority
     */
    public PowerBroker(Supplier<Double> prioritySupplier) {
        this(centralBank, prioritySupplier);
    }

    /**
     * Construct a broker with a constant priority.
     * 
     * @param priority, cannot be less than 1.0
     */
    public PowerBroker(double priority) {
        this(centralBank, () -> priority);
    }

    /**
     * Opens a new account at a given bank.
     * This constructor shouldn't be used outside of its class therefore it's made private.
     * 
     * @param bank    the bank where to register this account.
     * @param prioritySupplier  a <code>Double</code> supplier that will return the consumer's priority
     */
    private PowerBroker(PowerBank bank, Supplier<Double> prioritySupplier) {
        this.prioritySupplier = prioritySupplier;
        this.powerRequested = 0;
        this.powerMinimum = 0;
        this.bank = bank;
        bank.registerConsumer(this);
    }

    /**
     * Checks status and returns if this consumer is active.
     * Active consumer is one that is awaiting an allocation or ready to 
     * consume power.
     * 
     * @return true if active, otherwise false
     */
    public boolean isActive() {
        return status == Status.awaiting || status == Status.consuming;
    }

    /**
     * Sets the status of this consumer to disconnected.
     * Should only be used by the Power Bank.
     */
    public void disconnect() {
        powerAllowed = 0;
        status = Status.disconnected;
    }

    public void resetStatus() {
        if (powerRequested == 0.0) {
            disconnect();
        }
        else {
            status = Status.awaiting;
        }
    }

    /**
     * Obtains the current priority value from the consumer.
     * This is a wrapper for the priority supplier's {@code get()}
     * 
     * @return the priority value
     */
    public double getPriority() {
        double p = prioritySupplier.get();
        if (p < 1.0) p = 1.0;
        return p;
    }

    /**
     * Get the latest power allowence after the bank performed allocation.
     * Remeber: requested power is not immediately available until the bank
     * performs the power allocation which usually happens in a periodic loop.
     * @return  allowed amount of power (Watts)
     */
    public double getPowerAllowed() {
        synchronized(bank) {
            return this.powerAllowed;
        }
    }

    /**
     * Power Allowed setter, should only be used by the Power Bank.
     * @param power allowed (Watts)
     */
    public void allowPower(double powerAllowed) {
        this.powerAllowed = powerAllowed;
        status = Status.consuming;
    }

    /**
     * Reserves all minimum Power, should only be used by the Power Bank.
     * @param power allowed (Watts)
     */
    public void reserveMinPower() {
        powerAllowed = powerMinimum;
        status = Status.reserved;
    }

    /**
     * Requested power getter, should only be used by the Power Bank.
     * @return requested power (Watts)
     */
    public double getPowerRequested() {
        return powerRequested;
    }

    /**
     * Minimum power getter, should only be used by the Power Bank.
     * @return minimum considered power (Watts)
     */
    public double getPowerMinimum() {
        return powerMinimum;
    }

    /**
     * Returns as much power as available up to the requested value.
     * It is imperative to not exceed the allowed amount to avoid whole system brownouts.
     * Before calling this method precalculate the "wanted" amount as close as possible for the optimal power management.
     * After the allowance is returned recalculate and reapply control parameters to make sure
     * that the consumer won't exceed the allowed amount.
     * 
     * @param wattsWanted   amount of power wanted at the moment (Watts)
     * @param wattsLeast    lower threshold below which the power is too low to use at all
     * @return  allowed amount of power (Watts)
     */
    public double requestPower(double wattsWanted, double wattsLeast) {
        // Update the power requirements for reallocation
        // Can't request negative power
        this.powerRequested = wattsWanted >= 0 ? wattsWanted : 0;
        this.powerMinimum = wattsLeast >= 0 ? wattsLeast : 0;
        // and return what was allocated in the last allocation
        return getPowerAllowed();
    }

    /**
     * Returns as much power as available up to the requested value.
     * This is a shortcut overload for a single argument call in case if there's no lower threshold
     * @param wattsWanted   amount of power wanted at the moment (Watts)
     * @return  allowed amount of power (Watts)
     */
    public double requestPower(double wattsWanted) {
        return requestPower(wattsWanted, 0);
    }

    /**
     * Resets the power need on this account to zero.
     * This method should be called every time when the consumer goes dormant.
     */
    public void releasePower() {
        requestPower(0);
    }
}