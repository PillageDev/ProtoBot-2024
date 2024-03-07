package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;

public class PhoenixOdometryThread extends Thread {
    private final Lock signalsLock = new ReentrantLock();
    private BaseStatusSignal[] signals = new BaseStatusSignal[0];
    private final List<Queue<Double>> queues = new ArrayList<>();
    private boolean isCANFD = false;

    private static PhoenixOdometryThread instance = null;

    public static PhoenixOdometryThread getInstance() {
        if (instance == null) {
            instance = new PhoenixOdometryThread();
        }
        return instance;
    }

    private PhoenixOdometryThread() {
        setName("PhoenixOdometryThread");
        setDaemon(true);
        start();
    }
    
    public Queue<Double> registerSignal(ParentDevice device, StatusSignal<Double> signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(100);
        signalsLock.lock();
        Drive.odometryLock.lock();
        try {
            isCANFD = CANBus.isNetworkFD(device.getNetwork());
            BaseStatusSignal[] newSignals = new BaseStatusSignal[signals.length + 1];
            System.arraycopy(signals, 0, newSignals, 0, signals.length);
            newSignals[signals.length] = signal;
            signals = newSignals;
            queues.add(queue);
        } finally {
            Drive.odometryLock.unlock();
            signalsLock.unlock();
        }
        return queue;
    }

    @Override
    public void run() {
        while (true) {
            signalsLock.lock();
            try { 
                if (isCANFD) {
                    BaseStatusSignal.waitForAll(2.0 / Module.ODOMETRY_FREQUENCY, signals);
                } else {
                    Thread.sleep((long) (1000.0 / Module.ODOMETRY_FREQUENCY));
                    if (signals.length > 0) BaseStatusSignal.refreshAll(signals);
                }
            } catch (InterruptedException e) {
                e.printStackTrace();
            } finally {
                signalsLock.unlock();
            }

            Drive.odometryLock.lock();
            try {
                for (int i = 0; i < signals.length; i++) {
                    queues.get(i).offer(signals[i].getValueAsDouble());
                }
            } finally {
                Drive.odometryLock.unlock();
            }
        }
    }
}
