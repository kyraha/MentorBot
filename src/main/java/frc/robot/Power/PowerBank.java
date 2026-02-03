package frc.robot.Power;

import java.util.List;

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
     * Allocates power among consumers (brokers).
     * 
     * This method is expected to be called periodically to keep the allocation up to date.
     * When consumers put requests with their brokers the new allocation is not available until
     * this method is called. So make sure it's called regularly and at least as often as
     * the consumers change their power demand. Likely spot would be the Robot's periodic().
     * 
     * The logic in this method is the meat of the Power Bank class. Here are the formulas:
     * 
     * \[ Priority_{avg} = \frac{\sum_i^N {(Requested_i \times Priority_i)}} {\sum_i^N {Requested_i}} \]
     *
     * \[ Excess = \sum_i^N {Requested_i} - MaxPower \]
     *
     * \[ Power_{allowed} = Requested - \frac 1 {Priority} \times \frac {Excess \times Priority_{avg}} {N} \]
     */
    public synchronized void allocatePower() {
        // We make this method synchronized because the entire bank must be locked
        // while all accounts are being recalculated and modified

        // Reset all accounts with non-zero requests as consuming then start reallocation
        consumers.forEach(c -> {
            c.isConsuming = c.powerRequested > 0.0;
            c.powerAllowed = 0.0;
        });
        boolean needToReallocate = true;

        // Iterate until all consumers either are allocated or were denied in previous iteration
        while (needToReallocate) {
            List<PowerBroker> activeConsumers = consumers.stream().filter(c -> c.isConsuming).toList();
            if (activeConsumers.size() == 0) return;

            needToReallocate = false; // for now
            double totalRequested = activeConsumers.stream().mapToDouble(c -> c.powerRequested).sum();

            if (totalRequested > maxPower) {
                // Limit allocation because requested exceeds the max
                double excess = totalRequested - maxPower;
                double sumRevPriorities = activeConsumers.stream().mapToDouble(c -> 1.0 / c.getPriority()).sum();
                double tax = excess / sumRevPriorities;
                // System.out.println("Tax="+tax+", Consumers: "+activeConsumers);

                for (PowerBroker c : activeConsumers) {
                    // Find how much power allowed and compare it to the minimum
                    // If not enough then exclude the consumer and need to reallocate
                    double allowed = c.powerRequested - tax / c.getPriority();
                    if (allowed >= c.powerMinimum) {
                        c.powerAllowed = allowed;
                    }
                    else {
                        c.isConsuming = false;
                        needToReallocate = true;
                    }
                }
            }
            else {
                // Max power is not exceeded - allocate as requested
                activeConsumers.forEach(c -> {
                    c.powerAllowed = c.powerRequested;
                    c.isConsuming = true;
                });
            }
        }
    }
}
