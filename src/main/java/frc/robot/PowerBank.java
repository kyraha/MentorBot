package frc.robot;

public class PowerBank {
    public static final double MAX_POWER = 12.0 * 60.0; // Max available power in watts

    class ConsumerAccount {
        public PowerBank parent;
        public double priority; // Higher number means higher priority
        public double powerRequested; // in watts

        public ConsumerAccount(PowerBank parent, double priority) {
            this.parent = parent;
            this.priority = priority;
            this.powerRequested = 0;
        }

        public double requestPower(double power) {
            this.powerRequested = power;
            return parent.allocatePower(this);
        }

        public void releasePower() {
            this.powerRequested = 0;
        }
    }
    private java.util.List<ConsumerAccount> consumers;

    public PowerBank() {
        consumers = new java.util.ArrayList<>();
    }

    public void registerConsumer(double priority) {
        consumers.add(new ConsumerAccount(this, priority));
    }

    public double allocatePower(ConsumerAccount requester) {
        double totalRequested = consumers.stream().mapToDouble(c -> c.powerRequested).sum();
        if (totalRequested <= MAX_POWER) {
            return requester.powerRequested; // Full allocation
        } else {
            // Proportional allocation based on priority
            double totalPriority = consumers.stream().mapToDouble(c -> c.priority).sum();
            double allocatedPower = (requester.priority / totalPriority) * MAX_POWER;
            return Math.min(allocatedPower, requester.powerRequested);
        }
    }
}
