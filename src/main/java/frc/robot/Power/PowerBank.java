package frc.robot.Power;

/**
 * Power Bank is a class for managing power consumption among multiple consumers when
 * the power is scarce as it usually is in an FRC robot during a match.
 * <P>
 * For usage see {@link PowerBroker} class.
 */
public class PowerBank {
    // This global singleton centralBank is The One central power bank for the robot.
    // Define the maximum available power here in Watts
    public static final PowerBank centralBank = new PowerBank(12.0 * 60.0);

    private java.util.List<PowerBroker> consumers;
    private final double maxPower;
    public double getMaxPower() {return maxPower;}

    /**
     * Constructs a Power Bank.
     * Instantiates a container where it will store consumer accounts,
     * and defines the maximum total available power the bank will manage.
     * 
     * @param maxPower  Maximum total available power in the system (Watts)
     */
    private PowerBank(double maxPower) {
        this.consumers = new java.util.ArrayList<>();
        this.maxPower = maxPower;
    }

    /**
     * Opens an account at this bank that then will be used for power management.
     * 
     * @param broker the power broker object that shall be linked to the account
     */
    public void registerConsumer(PowerBroker broker) {
        consumers.add(broker);
    }

    /**
     * Serves a request for power by a consumer. Calculates the max available power for the given
     * requester and returns the amount allowed to use by the requester.
     * This method should not be used directly from the robot code.
     * Use {@link PowerBroker#requestPower} instead.
     * <P>
     * The current logic is really straightforward and simple.
     * All amounts of power being requested so far are multiplied by the corresponding priorities
     * and summed together - that's the total weight of the system. Then the requester weight is calulated
     * in the same manner, and then divided by the total weight - that's the ratio of power that
     * the requester can get at most at this moment.
     * 
     * @param requester the account manager of the requester, a PowerBroker
     * @return  the amount of power allowed to draw (Watts)
     */
    public double allocatePower(PowerBroker requester) {
        double totalRequested = consumers.stream().mapToDouble(c -> c.powerRequested).sum();
        if (totalRequested <= maxPower) {
            // Full allocation
            return requester.powerRequested;
        } else {
            // Proportional allocation based on priority
            double wantedWeight = requester.getPriority() * requester.powerRequested;
            double totalWeight = consumers.stream().mapToDouble(c -> c.getPriority() * c.powerRequested).sum();
            double allowedPower = (wantedWeight / totalWeight) * maxPower;
            return Math.min(allowedPower, requester.powerRequested);
        }
    }
}
