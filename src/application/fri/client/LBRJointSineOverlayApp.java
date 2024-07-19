package application.fri.client;

import java.util.logging.Logger;

import com.kuka.connectivity.fastRobotInterface.clientSDK.base.ClientApplication;
import com.kuka.connectivity.fastRobotInterface.clientSDK.connection.UdpConnection;

/**
 * Implementation of a FRI client application.
 * <p>
 * The application provides a {@link ClientApplication#connect}, a {@link ClientApplication#step()} and a
 * {@link ClientApplication#disconnect} method, which will be called successively in the application life-cycle.
 * 
 * 
 * @see ClientApplication#connect
 * @see ClientApplication#step()
 * @see ClientApplication#disconnect
 */
public class LBRJointSineOverlayApp
{
//    private LBRJointSineOverlayApp()
//    {
//        // only for sonar
//    }

    private static final int DEFAULT_PORTID = 30200;
    private static final double DEFAULT_FREQUENCY = 0.25;
    private static final double DEFAULT_AMPLITUDE = 0.04;
    private static final double DEFAULT_FILTER_COEFFICIENT = 0.99;
    private static final int DEFAULT_JOINTMASK = 0x8;

    public static void main(String[] args) {
        if (args.length > 0) {
            if ("help".equals(args[0])) {
                Logger.getAnonymousLogger().info("\nKUKA LBR joint sine overlay test application\n\n\tCommand line arguments:");
                Logger.getAnonymousLogger().info("\t1) remote hostname (optional)");
                Logger.getAnonymousLogger().info("\t2) port ID (optional)");
                Logger.getAnonymousLogger().info("\t3) bit mask encoding of joints to be overlaid (optional)");
                Logger.getAnonymousLogger().info("\t4) sine frequency in Hertz (optional)");
                Logger.getAnonymousLogger().info("\t5) sine amplitude in radians (optional)");
                Logger.getAnonymousLogger().info("\t6) filter coefficient from 0 (off) to 1 (optional)");
                return;
            }
        }

        String hostname = (args.length >= 1) ? args[0] : "172.32.1.201";
        int port = (args.length >= 2) ? Integer.valueOf(args[1]) : DEFAULT_PORTID;
        int jointMask = (args.length >= 3) ? Integer.valueOf(args[2]) : DEFAULT_JOINTMASK;
        double frequency = (args.length >= 4) ? Double.valueOf(args[3]) : DEFAULT_FREQUENCY;
        double amplitude = (args.length >= 5) ? Double.valueOf(args[4]) : DEFAULT_AMPLITUDE;
        double filterCoeff = (args.length >= 6) ? Double.valueOf(args[5]) : DEFAULT_FILTER_COEFFICIENT;

        Logger.getAnonymousLogger().info("Hostname: " + hostname);
        Logger.getAnonymousLogger().info("Enter LBRJointSineOverlay Client Application");

        /***************************************************************************/
        /*                                                                         */
        /* Place user Client Code here */
        /*                                                                         */
        /**************************************************************************/

        // create new sine overlay client
        LBRJointSineOverlayClient client = new LBRJointSineOverlayClient(jointMask, frequency, amplitude, filterCoeff);

        /***************************************************************************/
        /*                                                                         */
        /* Standard application structure */
        /* Configuration */
        /*                                                                         */
        /***************************************************************************/

        // create new udp connection
        UdpConnection connection = new UdpConnection();

        // pass connection and client to a new FRI client application
        ClientApplication app = new ClientApplication(connection, client);

        // connect client application to KUKA Sunrise controller
        app.connect(port, hostname);
        Logger.getAnonymousLogger().info("Connection: "+Boolean.toString(connection.isOpen()));
        
        /***************************************************************************/
        /*                                                                         */
        /* Standard application structure */
        /* Execution mainloop */
        /*                                                                         */
        /***************************************************************************/

        // repeatedly call the step routine to receive and process FRI packets
        boolean success = true;
        while (success) {
            success = app.step();
            Logger.getAnonymousLogger().info("Stepped");
        }

        /***************************************************************************/
        /*                                                                         */
        /* Standard application structure */
        /* Dispose */
        /*                                                                         */
        /***************************************************************************/

        // disconnect from controller
        app.disconnect();

        Logger.getAnonymousLogger().info("Exit LBRJointSineOverlay Client Application");
    }
}
