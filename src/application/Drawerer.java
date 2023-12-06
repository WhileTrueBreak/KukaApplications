package application;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.linRel;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.spl;

import java.util.Arrays;
import java.util.List;
import java.util.ListIterator;
import java.util.concurrent.TimeUnit;

import javax.inject.Inject;
import javax.inject.Named;

import com.kuka.common.Pair;
import com.kuka.common.ThreadUtil;
import com.kuka.core.geometry.Vector;
import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.math.geometry.Vector3D;
import com.kuka.nav.geometry.Vector2D;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.AbstractFrame;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.SPL;
import com.kuka.roboticsAPI.motionModel.Spline;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.task.ITaskLogger;

public class Drawerer extends RoboticsAPIApplication{
	@Inject
	private LBR robot;

	@Inject 
	private Gripper2F gripper2F1;

	@Inject
	private MediaFlangeIOGroup mF;

	@Inject
	@Named("RobotiqGripper")
	private Tool gripper;

	@Inject
	private ITaskLogger logger;
	
	private CartesianImpedanceControlMode springRobot;
	
	
	@Override
	public void initialize() {
		
		// Initializes the boing boing
		springRobot = new CartesianImpedanceControlMode(); 
		
		// Set stiffness

		// TODO: Stiff in every direction except plane perpendicular to flange
		springRobot.parametrize(CartDOF.X).setStiffness(1250);
		springRobot.parametrize(CartDOF.Y).setStiffness(5000);
		springRobot.parametrize(CartDOF.Z).setStiffness(5000);

		// Stiff rotation
		springRobot.parametrize(CartDOF.C).setStiffness(300);
		springRobot.parametrize(CartDOF.B).setStiffness(300);
		springRobot.parametrize(CartDOF.A).setStiffness(300);
		springRobot.setReferenceSystem(World.Current.getRootFrame());
		springRobot.parametrize(CartDOF.ALL).setDamping(0.4);
		
		// Inits the Robot
		gripper.attachTo(robot.getFlange());
		gripper2F1.initalise();
		gripper2F1.setSpeed(189);
//		gripper2F1.open();
//		mF.setLEDBlue(true);
//		ThreadUtil.milliSleep(10000);
		mF.setLEDBlue(false);
		gripper2F1.close();
		ThreadUtil.milliSleep(200);
	}

	private Vector2D[][] getPaths(){
//		Vector2D[][] path = {{new Vector2D(0.7, 0.6), new Vector2D(0.5,0.5)}};
//		Vector2D[][] path = {{new Vector2D(0.38854541181324487,0.009166666666666667),new Vector2D(0.38964857908065087,0.014208333333333297),new Vector2D(0.39192715712297327,0.02268216823117314),new Vector2D(0.39360891857396124,0.02799741088408856),new Vector2D(0.3952906800249492,0.03331265353700398),new Vector2D(0.39666666666666667,0.04555401197000455),new Vector2D(0.39666666666666667,0.05520042962408998),new Vector2D(0.39666666666666667,0.06484684727817533),new Vector2D(0.39951164274055995,0.0818802969441765),new Vector2D(0.40298883571976285,0.09305253999298137),new Vector2D(0.4064660286989657,0.10422478304178621),new Vector2D(0.4083410286989657,0.11387717120616345),new Vector2D(0.4071555023864295,0.11450229146937507),new Vector2D(0.4059699760738933,0.11512741173258662),new Vector2D(0.3979123122525481,0.11755462196732264),new Vector2D(0.38924958278344013,0.11989609199101049),new Vector2D(0.34464204577083457,0.13195317066558204),new Vector2D(0.33327830903960187,0.20137043351803102),new Vector2D(0.3729972817264704,0.21917595750637694),new Vector2D(0.4208491663346409,0.2406273652755419),new Vector2D(0.4463832046512743,0.2279139559960962),new Vector2D(0.4465934599983637,0.18253238690579082),new Vector2D(0.4466618380305644,0.1677736545570893),new Vector2D(0.4476307929977479,0.16601474881285386),new Vector2D(0.4612837293374545,0.15586572023912415),new Vector2D(0.4693231138063878,0.14988957410760587),new Vector2D(0.4844937264349986,0.13754703044659386),new Vector2D(0.49499620184547866,0.1284378454368752),new Vector2D(0.5140916116827151,0.11187569087375039),new Vector2D(0.5207131752236003,0.12005297570317576),new Vector2D(0.5255647019444396,0.12604435758887605),new Vector2D(0.5268999380784313,0.13147194170825893),new Vector2D(0.5257081848206792,0.14035712571462333),new Vector2D(0.5234301510438166,0.157341135742265),new Vector2D(0.5266509388631023,0.16086301825244767),new Vector2D(0.5432105390789658,0.15949568398293953),new Vector2D(0.5572876104267439,0.15833333333333335),new Vector2D(0.56368292120774,0.17500000000000002),new Vector2D(0.5700488619887224,0.19159012621116892),new Vector2D(0.569720319671599,0.21771929202393733),new Vector2D(0.5630942872925174,0.221814405244767),new Vector2D(0.5615525670843792,0.22276724073453907),new Vector2D(0.5610779092171435,0.22559706993795578),new Vector2D(0.5620394920319939,0.228102914585693),new Vector2D(0.5630010748468443,0.2306087592334303),new Vector2D(0.5623867279903345,0.23352492098007252),new Vector2D(0.5606742767953055,0.23458327402267581),new Vector2D(0.5589618256002764,0.2356416270652791),new Vector2D(0.5582476318078595,0.24010085134663756),new Vector2D(0.5590871794788235,0.2444926613145835),new Vector2D(0.5611108613711409,0.2550788693573341),new Vector2D(0.5538557528420853,0.26828133081838923),new Vector2D(0.535586779877988,0.28725744130469283),new Vector2D(0.5221364421904707,0.3012284014970373),new Vector2D(0.5210554805362577,0.30347947499859607),new Vector2D(0.5251824835166906,0.3089241079713595),new Vector2D(0.531498610383746,0.3172567873427033),new Vector2D(0.5311341357984224,0.32826373917340734),new Vector2D(0.5243601385754286,0.3337585176151928),new Vector2D(0.5205279368100572,0.33686703669953183),new Vector2D(0.5195226065420412,0.33699785455474995),new Vector2D(0.5212235285566785,0.33416666666666667),new Vector2D(0.5249260457810149,0.32800381979258675),new Vector2D(0.5210392879730646,0.3290743858035063),new Vector2D(0.5091216504588424,0.33750000000000013),new Vector2D(0.4928927996472413,0.34897358573608445),new Vector2D(0.49000000000000005,0.3523002118127163),new Vector2D(0.49000000000000005,0.35948927986778684),new Vector2D(0.49000000000000005,0.3686407829908224),new Vector2D(0.4864897039826403,0.3769507443423643),new Vector2D(0.48342422778092087,0.37505617585799783),new Vector2D(0.48202509729218795,0.37419146566126466),new Vector2D(0.48286639505627565,0.37982508101273343),new Vector2D(0.48529377836778226,0.3875753210834839),new Vector2D(0.487721161679289,0.39532556115423445),new Vector2D(0.4913242300029925,0.4080114128507016),new Vector2D(0.49330059686490135,0.4157661026311887),new Vector2D(0.49778882116314616,0.43337659174074544),new Vector2D(0.5267998737727203,0.46563577386999494),new Vector2D(0.5483365898385928,0.4769640241577069),new Vector2D(0.560696303789365,0.4834651979063241),new Vector2D(0.5632218673708311,0.48627327584834046),new Vector2D(0.5615600327909213,0.4916666666666667),new Vector2D(0.5430711628467932,0.5516712559340822),new Vector2D(0.5803630765044856,0.5686457379055778),new Vector2D(0.6289450803003347,0.522338899917989),new Vector2D(0.6424808330908989,0.50943704609272),new Vector2D(0.6436060247381755,0.5090354023320663),new Vector2D(0.667278413633668,0.508655591660546),new Vector2D(0.7021325954241875,0.5080963751721049),new Vector2D(0.7216237169104833,0.49167711887682036),new Vector2D(0.7320573184360856,0.45408605643772343),new Vector2D(0.7344620575903553,0.4454220587303089),new Vector2D(0.7409027995255745,0.4260760168034135),new Vector2D(0.7463700782921282,0.4110948521557336),new Vector2D(0.7590626805283502,0.37631521643062665),new Vector2D(0.7659520963612402,0.33978196368749797),new Vector2D(0.768209860455502,0.29528266996456),new Vector2D(0.7691944372049758,0.275877201484052),new Vector2D(0.770783529410687,0.243875),new Vector2D(0.7717411764681931,0.22416666666666668),new Vector2D(0.7734823529363862,0.18833333333333335),new Vector2D(0.7648921121213038,0.2016666666666667),new Vector2D(0.7563018713062214,0.21500000000000002),new Vector2D(0.7582147583187601,0.19833333333333333),new Vector2D(0.7606081856881765,0.1774797983908803),new Vector2D(0.7535850491642846,0.18672578750220653),new Vector2D(0.7447342304223932,0.2160805056573692),new Vector2D(0.7377971108467498,0.2390882291110025),new Vector2D(0.7359580333369061,0.23670041489748705),new Vector2D(0.7402222222222221,0.21022222222222278),new Vector2D(0.7441447074005743,0.18586581562027196),new Vector2D(0.7422125441157982,0.1807957725538861),new Vector2D(0.7351816392973058,0.19699561103962973),new Vector2D(0.732475257656117,0.2032313583011667),new Vector2D(0.730084062114956,0.20683333333333334),new Vector2D(0.729867871428059,0.20500000000000002),new Vector2D(0.7284834022282966,0.19325946866939656),new Vector2D(0.7310304468871476,0.15763631601855527),new Vector2D(0.7335972861743517,0.1528401329384558),new Vector2D(0.7355864028720324,0.1491234343912123),new Vector2D(0.7354336131570993,0.146298044316973),new Vector2D(0.7331677791688606,0.14489768189937674),new Vector2D(0.7311975145121665,0.1436799913747072),new Vector2D(0.7279551241424135,0.14721663268374738),new Vector2D(0.7257949390922412,0.15293961633859882),new Vector2D(0.714641116390592,0.18248946671163066),new Vector2D(0.7112432927289193,0.16776736880989435),new Vector2D(0.719623562744845,0.12620026320509947),new Vector2D(0.7230831889211763,0.10904011844229483),new Vector2D(0.7269186296195709,0.08750000000000001),new Vector2D(0.7281467642968328,0.07833333333333334),new Vector2D(0.7321312066353962,0.04859387930896636),new Vector2D(0.7311564814497734,0.049404152875867455),new Vector2D(0.7519076358603832,0.05858129815618232),new Vector2D(0.7750132302046553,0.06879968851122632),new Vector2D(0.7905508038453846,0.0698168610825808),new Vector2D(0.7821760192072225,0.060562828222044236),new Vector2D(0.7791378753767837,0.05720571707750187),new Vector2D(0.7775638116469892,0.05354729946412676),new Vector2D(0.778678099807679,0.052433011303432885),new Vector2D(0.7797923879683688,0.05131872314273902),new Vector2D(0.7734206606682869,0.04829274156823042),new Vector2D(0.7645187058074969,0.045708607804524876),new Vector2D(0.7465837839891424,0.040502308992073216),new Vector2D(0.7423911338854451,0.030393992111004974),new Vector2D(0.7495951584719405,0.009728513657293453),new Vector2D(0.7529865365972609,0.0),new Vector2D(0.6539051820551182,0.0),new Vector2D(0.5548238275129754,0.0),new Vector2D(0.5561372289483301,0.013772314759448345),new Vector2D(0.55704927078771,0.0233359766739315),new Vector2D(0.5563122191104609,0.026841052658868516),new Vector2D(0.5537253151918423,0.025242258111531822),new Vector2D(0.551676391836329,0.02397595383748126),new Vector2D(0.55,0.02015109339808305),new Vector2D(0.55,0.016742568246202533),new Vector2D(0.55,-0.00032452078568970445),new Vector2D(0.5516233914240429,0.0),new Vector2D(0.46624653055866916,0.0),new Vector2D(0.3865396531452339,0.0),new Vector2D(0.38854541181324487,0.009166666666666667)}};
		Vector2D[][] path = {{new Vector2D(0.38854541181324487,0.009166666666666667),new Vector2D(0.38964857908065087,0.014208333333333297),new Vector2D(0.39192715712297327,0.02268216823117314),new Vector2D(0.39360891857396124,0.02799741088408856),new Vector2D(0.3952906800249492,0.03331265353700398),new Vector2D(0.39666666666666667,0.04555401197000455),new Vector2D(0.39666666666666667,0.05520042962408998),new Vector2D(0.39666666666666667,0.06484684727817533),new Vector2D(0.39951164274055995,0.0818802969441765),new Vector2D(0.40298883571976285,0.09305253999298137),new Vector2D(0.4064660286989657,0.10422478304178621),new Vector2D(0.4083410286989657,0.11387717120616345),new Vector2D(0.4071555023864295,0.11450229146937507),new Vector2D(0.4059699760738933,0.11512741173258662),new Vector2D(0.3979123122525481,0.11755462196732264),new Vector2D(0.38924958278344013,0.11989609199101049),new Vector2D(0.34464204577083457,0.13195317066558204),new Vector2D(0.33327830903960187,0.20137043351803102),new Vector2D(0.3729972817264704,0.21917595750637694),new Vector2D(0.4208491663346409,0.2406273652755419),new Vector2D(0.4463832046512743,0.2279139559960962),new Vector2D(0.4465934599983637,0.18253238690579082),new Vector2D(0.4466618380305644,0.1677736545570893),new Vector2D(0.4476307929977479,0.16601474881285386),new Vector2D(0.4612837293374545,0.15586572023912415),new Vector2D(0.4693231138063878,0.14988957410760587),new Vector2D(0.4844937264349986,0.13754703044659386),new Vector2D(0.49499620184547866,0.1284378454368752),new Vector2D(0.5140916116827151,0.11187569087375039),new Vector2D(0.5207131752236003,0.12005297570317576),new Vector2D(0.5255647019444396,0.12604435758887605),new Vector2D(0.5268999380784313,0.13147194170825893),new Vector2D(0.5257081848206792,0.14035712571462333),new Vector2D(0.5234301510438166,0.157341135742265),new Vector2D(0.5266509388631023,0.16086301825244767),new Vector2D(0.5432105390789658,0.15949568398293953),new Vector2D(0.5572876104267439,0.15833333333333335),new Vector2D(0.56368292120774,0.17500000000000002),new Vector2D(0.5700488619887224,0.19159012621116892),new Vector2D(0.569720319671599,0.21771929202393733),new Vector2D(0.5630942872925174,0.221814405244767),new Vector2D(0.5615525670843792,0.22276724073453907),new Vector2D(0.5610779092171435,0.22559706993795578),new Vector2D(0.5620394920319939,0.228102914585693),new Vector2D(0.5630010748468443,0.2306087592334303),new Vector2D(0.5623867279903345,0.23352492098007252),new Vector2D(0.5606742767953055,0.23458327402267581),new Vector2D(0.5589618256002764,0.2356416270652791),new Vector2D(0.5582476318078595,0.24010085134663756),new Vector2D(0.5590871794788235,0.2444926613145835),new Vector2D(0.5611108613711409,0.2550788693573341),new Vector2D(0.5538557528420853,0.26828133081838923),new Vector2D(0.535586779877988,0.28725744130469283),new Vector2D(0.5221364421904707,0.3012284014970373),new Vector2D(0.5210554805362577,0.30347947499859607),new Vector2D(0.5251824835166906,0.3089241079713595),new Vector2D(0.531498610383746,0.3172567873427033),new Vector2D(0.5311341357984224,0.32826373917340734),new Vector2D(0.5243601385754286,0.3337585176151928),new Vector2D(0.5205279368100572,0.33686703669953183),new Vector2D(0.5195226065420412,0.33699785455474995),new Vector2D(0.5212235285566785,0.33416666666666667),new Vector2D(0.5249260457810149,0.32800381979258675),new Vector2D(0.5210392879730646,0.3290743858035063),new Vector2D(0.5091216504588424,0.33750000000000013),new Vector2D(0.4928927996472413,0.34897358573608445),new Vector2D(0.49000000000000005,0.3523002118127163),new Vector2D(0.49000000000000005,0.35948927986778684),new Vector2D(0.49000000000000005,0.3686407829908224),new Vector2D(0.4864897039826403,0.3769507443423643),new Vector2D(0.48342422778092087,0.37505617585799783),new Vector2D(0.48202509729218795,0.37419146566126466),new Vector2D(0.48286639505627565,0.37982508101273343),new Vector2D(0.48529377836778226,0.3875753210834839),new Vector2D(0.487721161679289,0.39532556115423445),new Vector2D(0.4913242300029925,0.4080114128507016),new Vector2D(0.49330059686490135,0.4157661026311887),new Vector2D(0.49778882116314616,0.43337659174074544),new Vector2D(0.5267998737727203,0.46563577386999494),new Vector2D(0.5483365898385928,0.4769640241577069),new Vector2D(0.560696303789365,0.4834651979063241),new Vector2D(0.5632218673708311,0.48627327584834046),new Vector2D(0.5615600327909213,0.4916666666666667),new Vector2D(0.5430711628467932,0.5516712559340822),new Vector2D(0.5803630765044856,0.5686457379055778),new Vector2D(0.6289450803003347,0.522338899917989),new Vector2D(0.6424808330908989,0.50943704609272),new Vector2D(0.6436060247381755,0.5090354023320663),new Vector2D(0.667278413633668,0.508655591660546),new Vector2D(0.7021325954241875,0.5080963751721049),new Vector2D(0.7216237169104833,0.49167711887682036),new Vector2D(0.7320573184360856,0.45408605643772343),new Vector2D(0.7344620575903553,0.4454220587303089),new Vector2D(0.7409027995255745,0.4260760168034135),new Vector2D(0.7463700782921282,0.4110948521557336),new Vector2D(0.7590626805283502,0.37631521643062665),new Vector2D(0.7659520963612402,0.33978196368749797),new Vector2D(0.768209860455502,0.29528266996456),new Vector2D(0.7691944372049758,0.275877201484052),new Vector2D(0.770783529410687,0.243875),new Vector2D(0.7717411764681931,0.22416666666666668),new Vector2D(0.7734823529363862,0.18833333333333335),new Vector2D(0.7648921121213038,0.2016666666666667),new Vector2D(0.7563018713062214,0.21500000000000002),new Vector2D(0.7582147583187601,0.19833333333333333),new Vector2D(0.7606081856881765,0.1774797983908803),new Vector2D(0.7535850491642846,0.18672578750220653),new Vector2D(0.7447342304223932,0.2160805056573692),new Vector2D(0.7377971108467498,0.2390882291110025),new Vector2D(0.7359580333369061,0.23670041489748705),new Vector2D(0.7402222222222221,0.21022222222222278),new Vector2D(0.7441447074005743,0.18586581562027196),new Vector2D(0.7422125441157982,0.1807957725538861),new Vector2D(0.7351816392973058,0.19699561103962973),new Vector2D(0.732475257656117,0.2032313583011667),new Vector2D(0.730084062114956,0.20683333333333334),new Vector2D(0.729867871428059,0.20500000000000002),new Vector2D(0.7284834022282966,0.19325946866939656),new Vector2D(0.7310304468871476,0.15763631601855527),new Vector2D(0.7335972861743517,0.1528401329384558),new Vector2D(0.7355864028720324,0.1491234343912123),new Vector2D(0.7354336131570993,0.146298044316973),new Vector2D(0.7331677791688606,0.14489768189937674),new Vector2D(0.7311975145121665,0.1436799913747072),new Vector2D(0.7279551241424135,0.14721663268374738),new Vector2D(0.7257949390922412,0.15293961633859882),new Vector2D(0.714641116390592,0.18248946671163066),new Vector2D(0.7112432927289193,0.16776736880989435),new Vector2D(0.719623562744845,0.12620026320509947),new Vector2D(0.7230831889211763,0.10904011844229483),new Vector2D(0.7269186296195709,0.08750000000000001),new Vector2D(0.7281467642968328,0.07833333333333334),new Vector2D(0.7321312066353962,0.04859387930896636),new Vector2D(0.7311564814497734,0.049404152875867455),new Vector2D(0.7519076358603832,0.05858129815618232),new Vector2D(0.7750132302046553,0.06879968851122632),new Vector2D(0.7905508038453846,0.0698168610825808),new Vector2D(0.7821760192072225,0.060562828222044236),new Vector2D(0.7791378753767837,0.05720571707750187),new Vector2D(0.7775638116469892,0.05354729946412676),new Vector2D(0.778678099807679,0.052433011303432885),new Vector2D(0.7797923879683688,0.05131872314273902),new Vector2D(0.7734206606682869,0.04829274156823042),new Vector2D(0.7645187058074969,0.045708607804524876),new Vector2D(0.7465837839891424,0.040502308992073216),new Vector2D(0.7423911338854451,0.030393992111004974),new Vector2D(0.7495951584719405,0.009728513657293453),new Vector2D(0.7529865365972609,0.0),new Vector2D(0.6539051820551182,0.0),new Vector2D(0.5548238275129754,0.0),new Vector2D(0.5561372289483301,0.013772314759448345),new Vector2D(0.55704927078771,0.0233359766739315),new Vector2D(0.5563122191104609,0.026841052658868516),new Vector2D(0.5537253151918423,0.025242258111531822),new Vector2D(0.551676391836329,0.02397595383748126),new Vector2D(0.55,0.02015109339808305),new Vector2D(0.55,0.016742568246202533),new Vector2D(0.55,-0.00032452078568970445),new Vector2D(0.5516233914240429,0.0),new Vector2D(0.46624653055866916,0.0),new Vector2D(0.3865396531452339,0.0),new Vector2D(0.38854541181324487,0.009166666666666667)},{new Vector2D(0.6007835107923717,0.25072457652168084),new Vector2D(0.5973410087848282,0.2578701621532163),new Vector2D(0.597104135436884,0.25763328880527214),new Vector2D(0.5968922676040241,0.24683333333333335),new Vector2D(0.5967203031788699,0.2380674526927751),new Vector2D(0.5975918078261064,0.23625847449277304),new Vector2D(0.6005579098550142,0.23922457652168083),new Vector2D(0.603435078466972,0.2421017451336385),new Vector2D(0.6034938710337457,0.24509869419696226),new Vector2D(0.6007835107923717,0.25072457652168084)}};
		return path;
	}

	private void penUp(){
		gripper.move(linRel(0,0, -20).setJointVelocityRel(0.2));
		logger.info("Moving Pen Up");
	}
	private void penDown(){
		gripper.move(linRel(0,0, 20).setMode(springRobot).setJointVelocityRel(0.2));
		logger.info("Moving Pen Down");
	}
	
	private Frame calibrateFrame(Tool grip){
		ForceCondition touch = ForceCondition.createSpatialForceCondition(gripper.getFrame("/TCP"), 10);
		IMotionContainer motion1 = gripper.move(linRel(0, 0, 150, gripper.getFrame("/TCP")).setCartVelocity(20).breakWhen(touch));
		if (motion1.getFiredBreakConditionInfo() == null){
			logger.info("No Collision Detected");
			return null;
		}
		else{
			logger.info("Collision Detected");
			return robot.getCurrentCartesianPosition(gripper.getFrame("/TCP"));
		}

	}

	private Vector3D frameToVector(Frame frame){
		return Vector3D.of(frame.getX(), frame.getY(), frame.getZ());
	}

	private Pair<Vector3D, Vector3D> getCanvasPlane(Vector3D origin, Vector3D up, Vector3D right){
		Vector3D ver = up.subtract(origin).normalize();
		Vector3D hor = right.subtract(origin).normalize();

		return new Pair<Vector3D, Vector3D>(hor, ver);
	}

	private Vector3D canvasToWorld(Vector2D point, Pair<Vector3D, Vector3D> canvas, double size){
		return canvas.getA().multiply(point.getX()*size).add(canvas.getB().multiply(point.getY()*size));
	}
	
	private Frame vectorToFrame(Vector3D vector, Frame baseFrame){
		return new Frame(vector.getX(), vector.getY(), vector.getZ(), baseFrame.getAlphaRad(), baseFrame.getBetaRad(), baseFrame.getGammaRad());
	}

	private Spline framesToSpline(Frame[] frames){
		SPL[] splines = new SPL[frames.length];
		for (int i=0;i<frames.length;i++){
			splines[i] = spl(frames[i]);
		}

		return new Spline(splines);
		// return new Spline((SPL[])Arrays.asList(frames).stream().map(x->spl(x)).collect(Collectors.toList()).toArray());
	}

	private void springyMove(Spline path){
		int vel = 40;
		gripper.move(path.setMode(springRobot).setCartVelocity(vel));
	}
	
	IMotionContainer m1;
	@Override
	public void run() {
		// Calibration sequence
		mF.setLEDBlue(true);
		logger.info("Moving to bottom left");
		gripper.move(ptp(getApplicationData().getFrame("/bottom_left")).setJointVelocityRel(0.2));
		logger.info("Calibrating point 1");
		Frame originFrame = calibrateFrame(gripper);
		penUp();
		Frame originUpFrame = robot.getCurrentCartesianPosition(gripper.getFrame("/TCP"));
		Vector3D origin = frameToVector(originFrame);
		logger.info(String.format("Origin: %s", origin.toString()));

		logger.info("Moving to bottom left");
		gripper.move(ptp(getApplicationData().getFrame("/bottom_left")).setJointVelocityRel(0.2));
		gripper.move(linRel(0, 40, 0).setJointVelocityRel(0.2));
		logger.info("Calibrating point 2");
		Vector3D up = frameToVector(calibrateFrame(gripper));
		logger.info(String.format("Up: %s", up.toString()));

		logger.info("Moving to bottom left");
		gripper.move(ptp(getApplicationData().getFrame("/bottom_left")).setJointVelocityRel(0.2));
		gripper.move(linRel(-40, 0,0).setJointVelocityRel(0.2));
		logger.info("Calibrating point 3");
		Vector3D right = frameToVector(calibrateFrame(gripper));
		logger.info(String.format("Right: %s", right.toString()));
		

		// get world unit vectors
		Pair<Vector3D,Vector3D> canvas = getCanvasPlane(origin, up, right);
		logger.info(String.format("Canvas X, Y: (%s), (%s)", canvas.getA().toString(), canvas.getB().toString()));

		// check upper right bound
		double diag_size = 25;
		gripper.move(ptp(getApplicationData().getFrame("/bottom_left")).setJointVelocityRel(0.2));
		Vector3D diag = canvas.getA().add(canvas.getB()).multiply(diag_size);
		logger.info("Diagonal vector: " + diag.toString());
		logger.info("Moving to top right");
		for(int i =0;i<10;i++){
			gripper.move(linRel(diag.getY(), diag.getZ(), diag.getX()).setCartVelocity(100));
		}
		logger.info(String.format("Found max at top right: %s", diag.toString()));

		// gets top right fraem
		Vector3D top_right = frameToVector(robot.getCurrentCartesianPosition(gripper.getFrame("/TCP")));
		double diag_mag = top_right.subtract(origin).length();
		double size = diag_mag/diag.length()*diag_size;
		logger.info(String.format("Canvas size: %f", size));
		mF.setLEDBlue(false);
		logger.info("Calibration completed.");
		
		Vector2D[][] paths = getPaths();
		Spline[] splines = new Spline[paths.length];

		logger.info("Creating Spline");
		Vector3D v = Vector3D.of(0,0,20);
		for (int i=0;i<paths.length;i++){
			Frame[] tempFrames = new Frame[paths[i].length];
			for (int j=0;j<paths[i].length;j++) {
				logger.info(i + "-" + j + " : " + paths[i][j].toString());
				Vector3D path3D = canvasToWorld(paths[i][j], canvas, size).add(origin).add(v);
				tempFrames[j] = vectorToFrame(path3D, originFrame);
				logger.info(i + "-" + j + " : " + path3D.toString());
			}

			splines[i] = framesToSpline(tempFrames);
			// Frame[] frames = (Frame[])Arrays.asList(path).stream().map(x->vectorToFrame(canvasToWorld(x, canvas, origin), originFrame)).collect(Collectors.toList()).toArray();
			// Spline spline = framesToSpline(frames);
		}

		logger.info("Start Drawing");
		// Spline[] splines = (Spline[])Arrays.asList(paths).stream().map(y-> framesToSpline((Frame[])Arrays.asList(y).stream().map(x->vectorToFrame(canvasToWorld(x, canvas, origin), originFrame)).collect(Collectors.toList()).toArray())).collect(Collectors.toList()).toArray();
		ListIterator<Spline> splineIterator = Arrays.asList(splines).listIterator();
		while(splineIterator.hasNext()){
			int index = splineIterator.nextIndex();
			logger.info("Start path "+index);
			gripper.move(lin(originUpFrame).setCartVelocity(100));
			Vector3D first = canvasToWorld(paths[index][0], canvas, size);
			logger.info("Moving to first frame");
			gripper.move(linRel(first.getY(), first.getZ(), first.getX()).setCartVelocity(100));
			logger.info("Start spline path");
			springyMove(splineIterator.next());
			logger.info("Finished path");
			penUp();
		}

		mF.setLEDBlue(true);
		//		ThreadUtil.milliSleep(120000);
	}
}
