package frc.robot;

import java.util.ArrayList;
import java.util.Random;

import org.apache.commons.math3.stat.regression.OLSMultipleLinearRegression;
import org.junit.jupiter.api.Test;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import edu.wpi.first.math.Vector;
import edu.wpi.first.cscore.OpenCvLoader;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N3;

public class TestHandEye {
    // Load OpenCV library before running tests
    static {
        OpenCvLoader.forceStaticLoad();
    }

    /**
     * Helper function to create a 4x4 homogeneous transformation matrix from
     * a rotation matrix and translation vector.
     */
    Mat makeHomogeneous(Mat r, Mat t) {
        Mat h = Mat.eye(4, 4, CvType.CV_64F);
        r.copyTo(h.submat(0, 3, 0, 3));
        t.copyTo(h.submat(0, 3, 3, 4));
        return h;
    }

    /**
     * This test demonstrates how to perform hand-eye calibration using OpenCV's calibrateHandEye()
     * function. The goal is to find the transformation from the camera frame to the robot frame
     * given a series of robot movements and corresponding visual measurements of a fixed target.
     * The measurements can be taken, e.g. from robot's odometry and AprilTag detections.
     * 
     * We simulate a robot with a camera mounted on it, looking at a fixed target (like an AprilTag).
     * The robot moves around, and we record:
     * - The robot's movement (odometry) as a series of rotation and translation pairs (bTg)
     * - The camera's observed transform to the target (visual measurement) as a series of rotation
     *   and translation pairs (cTt)
     * We assume the target's position in the world is fixed and known (bTt).
     * 
     * To simulate the vision measurements (cTt) the relationship is defined as:
     * cTt = gTc^-1 * bTg^-1 * bTt
     * where:
     * - gTc is the robot-to-camera transform (fixed, predefined for this test, but what we
     *   want to find in general)
     * - bTg is the robot movement (odometry)
     * - bTt is the world-to-target transform (fixed, predefined for this test)
     * 
     * Naming of the matrices comes from the OpenCV: Calib3d documentation:
     * https://docs.opencv.org/4.10.0/d9/d0c/group__calib3d.html#gaebfc1c9f7434196a374c382abf43439b
     * 
     * By collecting multiple pairs of (bTg, cTt), we can solve for gTc
     * 
     * gTc stands for "grip to camera", or for our case it'll be `Transform3d robotToCamera`
     * which is required by PhotonPoseEstimator.
     */
    @Test
    void testHandEyeModeled() {
        // Random number generator for simulating noise
        Random random = new Random();
        // Noise levels to simulate real-world inaccuracies
        double noiseLevel = 0.0254; // in meters
        double noiseAngle = 0.0175; // in radians, e.g. 0.017 ~= 1 degree

        // Fix robot2camera and world2target transforms to some arbitrary constant values
        // This is needed for simulation purposes and then for later verification
        // In real life, these would be unknown and what we want to find
        Mat r_robot2cam = Mat.eye(3, 3, CvType.CV_64F); // Eye in opencv-ese is I - identity matrix
        Mat t_robot2cam = Mat.zeros(3, 1, CvType.CV_64F);
        Mat r_world2target = Mat.eye(3, 3, CvType.CV_64F);
        Mat t_world2target = Mat.zeros(3, 1, CvType.CV_64F);

        // Let's say the camera is rotated 20 degrees looking down
        double angle = Math.toRadians(20);
        r_robot2cam.put(0, 0, Math.cos(angle));
        r_robot2cam.put(0, 2, Math.sin(angle));
        r_robot2cam.put(2, 0, -Math.sin(angle));
        r_robot2cam.put(2, 2, Math.cos(angle));
        // And the camera is 0.5m above the robot center, and 0.3m forward, 0.1m to the left
        t_robot2cam.put(2, 0, 0.5);
        t_robot2cam.put(0, 0, 0.3);
        t_robot2cam.put(1, 0, 0.1);

        // Let's say the target is 2m in front of the robot at the start
        t_world2target.put(0, 0, 2.0);
        // No rotation between world and target

        // Compute the fixed transforms that will be the same for all "measurements"
        Mat bTt = makeHomogeneous(r_world2target, t_world2target);
        Mat gTc = makeHomogeneous(r_robot2cam, t_robot2cam);
        System.out.println("bTt (world to target, predefined):\n" + TestHomography.matToString(bTt));
        System.out.println("gTc (robot to camera, predefined):\n" + TestHomography.matToString(gTc));

        // Simulate a series of odometry and visual measurements
        // In real life, these would be collected from the robot odometry and PhotonPoseEstimator
        ArrayList<Mat> r_odos = new ArrayList<>(); // Rotation part of odometry measurements
        ArrayList<Mat> t_odos = new ArrayList<>(); // Translation part of odometry measurements
        ArrayList<Mat> r_visuals = new ArrayList<>(); // Rotation part of visual measurements
        ArrayList<Mat> t_visuals = new ArrayList<>(); // Translation part of visual measurements
        for (int i = 0; i < 40; i++) {
            // To simulate movement, let's say the robot moves in some direction and rotates a bit
            double x = i * 0.02;     // Move +2cm in X each step
            double y = i * -0.02;   // Move -1cm in Y each step
            double z = 0;           // No movement in Z
            double theta = Math.toRadians(i); // Rotate a degrees each step

            // Create new rotation matrices and translation vectors to store the simulated data
            var r_odo = Mat.eye(3, 3, CvType.CV_64F);
            var t_odo = Mat.zeros(3, 1, CvType.CV_64F);
            var r_visual = Mat.eye(3, 3, CvType.CV_64F);
            var t_visual = Mat.zeros(3, 1, CvType.CV_64F);

            // Odometry measurement, just what we assumed above about how the robot moves
            t_odo.put(0, 0, x);
            t_odo.put(1, 0, y);
            t_odo.put(2, 0, z);
            r_odo.put(0, 0, Math.cos(theta));
            r_odo.put(0, 1, -Math.sin(theta));
            r_odo.put(1, 0, Math.sin(theta));
            r_odo.put(1, 1, Math.cos(theta));

            // Visual measurement needs to be modeled, let's use transformations:
            // cTt = bTg^-1 * gTc^-1 * bTt -- see the description of this test for details
            Mat bTg = makeHomogeneous(r_odo, t_odo);
            Mat cTt = gTc.inv().matMul(bTg.inv()).matMul(bTt);

            // Extract rotation and translation from cTt into separate matrices
            cTt.submat(0, 3, 0, 3).copyTo(r_visual);
            cTt.submat(0, 3, 3, 4).copyTo(t_visual);

            // Add some noise to the visual measurement
            // Add noise to translation in all 3 axes
            t_visual.put(0, 0, t_visual.get(0, 0)[0] + random.nextGaussian() * noiseLevel);
            t_visual.put(1, 0, t_visual.get(1, 0)[0] + random.nextGaussian() * noiseLevel);
            t_visual.put(2, 0, t_visual.get(2, 0)[0] + random.nextGaussian() * noiseLevel);
            // Add noise to rotation (small rotation in random direction)
            // Create a small rotation vector with random angles
            Mat v_noise = Mat.zeros(3, 1, CvType.CV_64F);
            v_noise.put(0, 0, random.nextGaussian() * noiseAngle);
            v_noise.put(1, 0, random.nextGaussian() * noiseAngle);
            v_noise.put(2, 0, random.nextGaussian() * noiseAngle);
            // Convert rotation vector to rotation matrix
            Mat r_noise = new Mat();
            Calib3d.Rodrigues(v_noise, r_noise);
            // Apply the noise rotation to the visual rotation
            r_visual = r_noise.matMul(r_visual);

            // Add the "measurements" to the lists
            r_odos.add(r_odo);
            t_odos.add(t_odo);
            r_visuals.add(r_visual);
            t_visuals.add(t_visual);
        }

        // Prepare empty matrices to receive the calibration result
        Mat r_calibrated = new Mat();
        Mat t_calibrated = new Mat();

        // Now perform hand-eye calibration to find the camera-to-robot transform
        Calib3d.calibrateHandEye(r_odos, t_odos, r_visuals, t_visuals, r_calibrated, t_calibrated);
        // System.out.println("Calibrated Rotation:\n" + r_cam2robot.dump());
        // System.out.println("Calibrated Translation:\n" + t_cam2robot.dump());
        Mat robotToCamera = makeHomogeneous(r_calibrated, t_calibrated);
        System.out.println("robotToCamera estimated with calibrateHandEye:\n" + TestHomography.matToString(robotToCamera));
        // Verify the result against the predefined robotToCamera (gTc) by inverting it
        System.out.println(
            "Difference (should be close to [identity|zero]):\n" + 
            TestHomography.matToString(robotToCamera.matMul(gTc.inv())));

        /* Note:
         * Since we simulated driving on a flat surface, which what is going to be in real life,
         * the offset of the camera in Z axis (height) cannot be determined. So the
         * calibrateHandEye() is leaving it at zero, which is acceptable since we don't
         * really care about the Z coordinate on a flat FRC field. And we can't measure it
         * with the robot's odometry anyway.
         */
    }

    @Test
    void testHandEyeReal() {
        // In this test, we use real collected data from the robot
        double[][] data = {
            {1.148387276141106, 0.3320249012409047, -0.009217855726489588}, {-0.7491196890621707, -0.6413112148571208, -0.1659506467534282, -0.6621593093935346, 0.7177081386939521, 0.2154995977628268, -0.01909817904603879, 0.2713207573167721, -0.9622994888319225},
            {4.56142631033084, 5.999624264551517, 0}, {0.5025448742458571, 0.864551125942946, 0, -0.864551125942946, 0.5025448742458571, -0, -0, 0, 1},
            {1.215735028439739, 0.1467555835820842, -0.06010012696456646}, {-0.732125396487161, -0.6579573578945943, -0.1763080231041186, -0.6810727268388339, 0.7027042442608162, 0.2057831039083393, -0.01150411121438147, 0.2707376225983378, -0.9625844353276067},
            {4.32063391954491, 6.010634853027435, 0}, {0.5203377440863128, 0.8539605565125165, 0, -0.8539605565125165, 0.5203377440863128, -0, -0, 0, 1},
            {1.217051026323719, 0.1469296016548234, -0.05998841961714652}, {-0.7341704330561916, -0.6565723976965587, -0.1729348484518849, -0.6788081119522251, 0.7043135557437006, 0.2077545723769292, -0.01460555969279854, 0.2699168423397421, -0.9627728578680452},
            {4.320640109496714, 6.010633176467266, 0}, {0.5202578947403229, 0.8540092054306951, 0, -0.8540092054306951, 0.5202578947403229, -0, -0, 0, 1},
            {1.118273303569723, 0.5088035139005933, 0.03575479113185032}, {-0.7296908452658699, -0.667128157765014, -0.1499709686980766, -0.682153534850881, 0.6951276242157873, 0.2268570936571372, -0.04709379179497694, 0.2678387708476387, -0.9623120946996342},
            {4.663296778827076, 6.01278547082554, 0}, {0.5440394764359437, 0.8390596212899917, 0, -0.8390596212899917, 0.5440394764359437, -0, -0, 0, 1},
            {1.593507283451724, 0.7934268547287584, 0.03239762440423954}, {-0.7779446300960369, -0.6189548238477579, -0.1081530329687137, -0.6231159784508488, 0.7378364661716961, 0.2594683537283595, -0.08079993752461787, 0.2692438954235631, -0.9596765574260755},
            {4.820552309717889, 6.504305701205765, 0}, {0.5113910040491816, 0.8593481488765597, 0, -0.8593481488765597, 0.5113910040491816, -0, -0, 0, 1},
            {1.590083465778465, 0.791601477429551, 0.02631543899393307}, {-0.7829680780323347, -0.6109565749110151, -0.117017316476464, -0.6183250616096421, 0.7437828210977806, 0.2538921684992649, -0.06808161989688502, 0.2711442026170358, -0.9601279677310666},
            {4.820212331886414, 6.503388430967583, 0}, {0.5118842088639437, 0.8590544550351475, 0, -0.8590544550351475, 0.5118842088639437, -0, -0, 0, 1},
            {1.672475404846742, 0.3355387723219567, -0.08755944882763678}, {-0.7618076288268221, -0.6253267782432212, -0.1691613344508958, -0.6477067913533132, 0.7307582607705866, 0.2155650174549268, -0.01118253531406499, 0.2737860199635847, -0.9617256189664749},
            {4.475418521388147, 6.431885237643833, 0}, {0.4815086426934851, 0.8764413425959992, 0, -0.8764413425959992, 0.4815086426934851, -0, -0, 0, 1},
            {1.612543048695852, 0.7807401222282286, 0.02918568630235674}, {-0.7771502160420423, -0.6171843383698539, -0.1229676143412281, -0.6257436201905313, 0.7370562181144175, 0.2553293033118612, -0.066951202354069, 0.2753754233947501, -0.9590025613592004},
            {4.884055479587222, 6.463370950736606, 0}, {0.4969937831578999, 0.8677541008271862, 0, -0.8677541008271862, 0.4969937831578999, -0, -0, 0, 1},
            {1.439738943865586, 0.7063203250590346, 0.02613061416967533}, {-0.7779141226033857, -0.6163378191927164, -0.1223818225349311, -0.6251684803502252, 0.7394925076674442, 0.2496301309543885, -0.06335604967795168, 0.2707000623134052, -0.9605766430809797},
            {4.881793689187255, 6.29902339201858, 0}, {0.5046861192901897, 0.8633029138117213, 0, -0.8633029138117213, 0.5046861192901897, -0, -0, 0, 1},
            {2.440695994675481, 0.715951461059011, -0.1192565527486509}, {-0.8202058248524084, -0.5519907215212672, -0.1502286531678032, -0.5720230312781618, 0.7880441966121654, 0.2275433933851519, -0.007215023598318424, 0.2725666662310668, -0.9621098460644497},
            {4.892180962223072, 7.219550404032384, 0}, {0.4041890300331846, 0.9146754768773641, 0, -0.9146754768773641, 0.4041890300331846, -0, -0, 0, 1},
            {2.597624434802547, 0.1540277508485643, -0.2708514153636759}, {-0.8226647170669199, -0.5418631849311178, -0.1720669988972601, -0.5681388006660125, 0.7947271075689115, 0.2136050740805016, 0.02100158258297466, 0.2734832962201837, -0.9616474511054214},
            {4.331059652895174, 7.25783903972323, 0}, {0.390906263672719, 0.9204304933135553, 0, -0.9204304933135553, 0.390906263672719, -0, -0, 0, 1},
            {1.100134254512579, -0.4800080377970353, -0.1757328944717054}, {-0.6517856377218052, 0.738057056415356, -0.174491443730289, 0.6909886455360058, 0.6727462769118675, 0.264475213661956, 0.312586266812694, 0.05180953943745405, -0.9484754068627099},
            {6.038109025466015, 5.733938746297362, 0}, {-0.8556481838268317, 0.5175579054693727, 0, -0.5175579054693727, -0.8556481838268317, -0, -0, 0, 1},
            {1.093241862176189, -0.4831897244753377, -0.1772530839696512}, {-0.6596128722040582, 0.7317141400639405, -0.1717709988711748, 0.6877103085216744, 0.679776228056955, 0.2548701852349198, 0.3032579601193292, 0.04998696829321393, -0.9515965072577333},
            {6.037337021437418, 5.733954763230952, 0}, {-0.8550300768887003, 0.5185784102869145, 0, -0.5185784102869145, -0.8550300768887003, -0, -0, 0, 1},
            {1.003466960289849, -0.1935826546045173, -0.1010302892234708}, {-0.6891328862996362, 0.7055069652468692, -0.16539584942944, 0.6613116819419441, 0.705623505063754, 0.2544844365155938, 0.2962477415060745, 0.06599538688192812, -0.9528283604946144},
            {6.023298914402976, 5.488578417405829, 0}, {-0.8279911674355205, 0.5607411404995751, 0, -0.5607411404995751, -0.8279911674355205, -0, -0, 0, 1},
            {1.005907445749904, -0.1989398620056868, -0.1020145394692833}, {-0.6912948866040229, 0.7032256022707379, -0.1660877240077942, 0.6601162600792452, 0.7081136181912457, 0.2506424283937037, 0.2938671518460707, 0.06363062189748331, -0.9537259779532159},
            {6.023300065073086, 5.488583050096324, 0}, {-0.8277208036309511, 0.5611401529355496, 0, -0.5611401529355496, -0.8277208036309511, -0, -0, 0, 1},
            {1.431332442782082, -0.03272193972817544, -0.1272407577748866}, {-0.6490198632842155, 0.7455695371423334, -0.1513250882963742, 0.7054801358034442, 0.6642690452509968, 0.2470716768633816, 0.2847296876851668, 0.05359758209747946, -0.9571083032468226},
            {6.494793241822519, 5.426179537988479, 0}, {-0.8585508638103616, 0.5127284020321888, 0, -0.5127284020321888, -0.8585508638103616, -0, -0, 0, 1},
            {2.093641218843965, -0.3766575218389611, -0.3046654033558114}, {-0.5595276366158635, 0.815145971085074, -0.1498861891130674, 0.7773861035518937, 0.5788674890904402, 0.2461367020144161, 0.2874015829244349, 0.02120084663169955, -0.9575755083723836},
            {7.20158265107685, 5.717752324840912, 0}, {-0.9117048619131918, 0.4108457676109707, 0, -0.4108457676109707, -0.9117048619131918, -0, -0, 0, 1},
            {2.090442583939112, -0.5111337173965134, -0.3362783841544688}, {-0.5519141236040637, 0.818941027679091, -0.1572462824688486, 0.7795593479545245, 0.5736495165312834, 0.2514228613315455, 0.2961047503554701, 0.01618101873613735, -0.9550184037229785},
            {7.190099681223873, 5.832234273297404, 0}, {-0.9144004952186482, 0.4048107389186842, 0, -0.4048107389186842, -0.9144004952186482, -0, -0, 0, 1},
            {1.372731327136103, 0.4686379319123664, -0.008251335292531525}, {-0.9728949164675247, 0.1398047940661062, -0.1842012515369589, 0.09520803057595476, 0.9680723937877806, 0.231886332714639, 0.2107389675060036, 0.2080635959079094, -0.9551432497967902},
            {6.047495125421839, 5.812494981502267, 0}, {-0.316273881110023, 0.9486679251074124, 0, -0.9486679251074124, -0.316273881110023, -0, -0, 0, 1},
            {1.411752397171106, 0.5294401445876504, -0.001368372790965466}, {-0.9725936462283564, 0.1409196409253229, -0.1849412180064504, 0.096151635815356, 0.9679731464313208, 0.2319112992458107, 0.2116989897147894, 0.2077730554940338, -0.9549941859322553},
            {6.118834869776534, 5.812983847555269, 0}, {-0.3155898446985062, 0.9488957002343158, 0, -0.9488957002343158, -0.3155898446985062, -0, -0, 0, 1},
            {1.323819287533174, 0.5709725275880129, 0.02091993569909767}, {-0.9735214120335396, 0.1360263216152107, -0.183719623720656, 0.09165705880245117, 0.9685281368841935, 0.231413551105628, 0.2094159590000442, 0.2084468466815182, -0.9553506519726798},
            {6.127272360409666, 5.715894359885922, 0}, {-0.3126329060337056, 0.9498740264187248, 0, -0.9498740264187248, -0.3126329060337056, -0, -0, 0, 1},
            {2.20906862609101, -0.01736974213321762, -0.2457611850340337}, {-0.9433823446748453, 0.2434020097144288, -0.225355748590465, 0.1881671769522065, 0.9521789569841473, 0.2407246298043021, 0.2731718603253798, 0.1846908106635476, -0.9440690860222112},
            {6.323067437070851, 6.638537356728153, 0}, {-0.4169957231440395, 0.9089084480185997, 0, -0.9089084480185997, -0.4169957231440395, -0, -0, 0, 1},
            {1.944337148492108, -0.2942677314297241, -0.2670952876590267}, {-0.9326533315637113, 0.2751779066668441, -0.2333128432074942, 0.216716602120389, 0.9443440417287369, 0.2474838281925238, 0.2884296751177037, 0.1802538502609231, -0.9403812375720868},
            {5.833500372425688, 6.627158593129533, 0}, {-0.3710841865321257, 0.9285992281419313, 0, -0.9285992281419313, -0.3710841865321257, -0, -0, 0, 1},
            {1.9488767084572, -0.2917636153810581, -0.2669844562152952}, {-0.9444346511033148, 0.2645325516253945, -0.1951043795662171, 0.2152773826001959, 0.9463438948901545, 0.2410163503684783, 0.2483925086152346, 0.1856226326035977, -0.9507078415208012},
            {5.833417793686698, 6.627156959309348, 0}, {-0.3709605223927641, 0.9286486369053085, 0, -0.9286486369053085, -0.3709605223927641, -0, -0, 0, 1},
            {1.575210276888317, 0.1006710062645592, -0.1251816470014165}, {-0.9629234801259698, 0.1100858686364329, -0.2462914390486469, 0.05160443306141663, 0.9712576356485642, 0.2323695067834219, 0.2647930397697438, 0.2110443240670619, -0.9409277014566932},
            {5.836509829273421, 6.166468426980619, 0}, {-0.2890906769852424, 0.9573017186238694, 0, -0.9573017186238694, -0.2890906769852424, -0, -0, 0, 1},
            {1.576093372138302, 0.1007186994988151, -0.125258684198028}, {-0.962999425097381, 0.09614097933400839, -0.2517717604395158, 0.03652280878753086, 0.9721432207432582, 0.2315246051744702, 0.2670172123672641, 0.2137626498142803, -0.9396847012929359},
            {5.83651022669588, 6.166459205917855, 0}, {-0.2885838772443441, 0.9574546181384375, 0, -0.9574546181384375, -0.2885838772443441, -0, -0, 0, 1},
            {1.739512920179012, -0.3528395376590727, -0.2486713596432746}, {-0.9744231796970759, 0.1806553999231663, -0.1336528837984466, 0.1471187624588566, 0.9624166629406352, 0.2282766668470219, 0.1698691749600696, 0.2027752286962403, -0.9643789037642687},
            {5.582613190374987, 6.524226355121064, 0}, {-0.3162776574187522, 0.9486666661255189, 0, -0.9486666661255189, -0.3162776574187522, -0, -0, 0, 1},
            {1.739765287272655, -0.3528027733301358, -0.2485765054059024}, {-0.971171821726428, 0.2018070444302306, -0.1268826603713317, 0.1695651299072818, 0.9589414239226084, 0.227330183224329, 0.1675498713938582, 0.1992617933865278, -0.965516327304871},
            {5.582609374064954, 6.524231833233598, 0}, {-0.315591761674614, 0.9488950626719024, 0, -0.9488950626719024, -0.315591761674614, -0, -0, 0, 1},
            {1.084296352273975, 0.2296750652935435, -0.01846710473421753}, {-0.9755503161159913, 0.04109595644780682, -0.2158997524075233, -0.009020605944881008, 0.9740467327664496, 0.2261671750174897, 0.2195910048120757, 0.2225850056734384, -0.9498608876330148},
            {5.667726628049202, 5.69851320942764, 0}, {-0.2254049934787767, 0.974265153289818, 0, -0.974265153289818, -0.2254049934787767, -0, -0, 0, 1}
        };

        ArrayList<Mat> r_odos = new ArrayList<>(); // Rotation part of odometry measurements
        ArrayList<Mat> t_odos = new ArrayList<>(); // Translation part of odometry measurements
        ArrayList<Mat> r_visuals = new ArrayList<>(); // Rotation part of visual measurements
        ArrayList<Mat> t_visuals = new ArrayList<>(); // Translation part of visual measurements

        System.out.println("Loaded " + (data.length / 4) + " measurement pairs from real robot data.");
        for (int i = 0; i < data.length / 4; i++) {
            // Visual measurement
            Mat t_vis = Mat.zeros(3, 1, CvType.CV_64F);
            t_vis.put(0, 0, data[i * 4 + 0]);
            t_visuals.add(t_vis);
            Mat r_vis = Mat.zeros(3, 3, CvType.CV_64F);
            r_vis.put(0, 0, data[i * 4 + 1]);
            r_visuals.add(r_vis);

            // Odometry measurement
            Mat t_odo = Mat.zeros(3, 1, CvType.CV_64F);
            t_odo.put(0, 0, data[i * 4 + 2]);
            t_odos.add(t_odo);
            Mat r_odo = Mat.zeros(3, 3, CvType.CV_64F);
            r_odo.put(0, 0, data[i * 4 + 3]);
            r_odos.add(r_odo);
        }

        // Prepare empty matrices to receive the calibration result
        Mat r_calibrated = new Mat();
        Mat t_calibrated = new Mat();

        // Now perform hand-eye calibration to find the camera-to-robot transform
        Calib3d.calibrateHandEye(r_odos, t_odos, r_visuals, t_visuals, r_calibrated, t_calibrated);
        // System.out.println("Calibrated Rotation:\n" + r_cam2robot.dump());
        // System.out.println("Calibrated Translation:\n" + t_cam2robot.dump());
        double[] r_buffer = new double[9];
        r_calibrated.get(0, 0, r_buffer);
        double[] t_buffer = new double[3];
        t_calibrated.get(0, 0, t_buffer);
        Matrix<N3,N3> r_matrix = new Matrix<N3,N3>(N3.instance, N3.instance, r_buffer);
        Rotation3d r_result = new Rotation3d(r_matrix);
        System.out.println("robotToCamera Rotation estimated with calibrateHandEye:\n" + r_result.getAxis() + r_result.getAngle());
        System.out.println("robotToCamera: " + r_result.getX() + "," + r_result.getY() + "," + r_result.getZ() + "," + r_result.getAngle());

        Mat robotToCamera = makeHomogeneous(r_calibrated, t_calibrated);
        System.out.println("robotToCamera estimated with calibrateHandEye:\n" + TestHomography.matToString(robotToCamera));
    }
}
