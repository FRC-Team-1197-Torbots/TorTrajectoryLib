package org.usfirst.frc.team1197.TorTrajectoryLib.spline_generator;

import org.apache.commons.math3.analysis.UnivariateFunction;

public class GaussLagrange {
	
	// Apologies for the enormous MATLAB output copypasta, this is just until I dig up my AMS 147 notes.
	// Unless it turns out we would need a symbolic library if we wanted to compute these in Java.
	// Then these numbers ain't going anywhere.
	
	private static final double points[][] = {
		{ 0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,
		  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,
		  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,
		  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D},
		{ 0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,
		  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,
		  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,
		  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D},
		{-0.577350269189626D,  0.577350269189626D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,
		  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,
		  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,
		  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D},
		{-0.774596669241483D,  0.000000000000000D,  0.774596669241483D,  0.000000000000000D,  0.000000000000000D,
		  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,
		  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,
		  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D},
		{-0.861136311594053D, -0.339981043584856D,  0.339981043584856D,  0.861136311594053D,  0.000000000000000D,
		  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,
		  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,
		  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D},
		{-0.906179845938664D, -0.538469310105683D,  0.000000000000000D,  0.538469310105683D,  0.906179845938664D,
		  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,
		  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,
		  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D},
		{-0.932469514203152D, -0.661209386466264D, -0.238619186083197D,  0.238619186083197D,  0.661209386466264D,
		  0.932469514203152D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,
		  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,
		  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D},
		{-0.949107912342758D, -0.741531185599394D, -0.405845151377397D,  0.000000000000000D,  0.405845151377397D,
		  0.741531185599394D,  0.949107912342758D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,
		  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,
		  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D},
		{-0.960289856497536D, -0.796666477413627D, -0.525532409916329D, -0.183434642495650D,  0.183434642495650D,
		  0.525532409916329D,  0.796666477413627D,  0.960289856497536D,  0.000000000000000D,  0.000000000000000D,
		  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,
		  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D},
		{-0.968160239507626D, -0.836031107326636D, -0.613371432700590D, -0.324253423403809D,  0.000000000000000D,
		  0.324253423403809D,  0.613371432700590D,  0.836031107326636D,  0.968160239507626D,  0.000000000000000D,
		  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,
		  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D},
		{-0.973906528517172D, -0.865063366688985D, -0.679409568299024D, -0.433395394129247D, -0.148874338981631D,
		  0.148874338981631D,  0.433395394129247D,  0.679409568299024D,  0.865063366688985D,  0.973906528517172D,
		  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,
		  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D},
		{-0.978228658146057D, -0.887062599768095D, -0.730152005574049D, -0.519096129206812D, -0.269543155952345D,
		  0.000000000000000D,  0.269543155952345D,  0.519096129206812D,  0.730152005574049D,  0.887062599768095D,
		  0.978228658146057D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,
		  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D},
		{-0.981560634246719D, -0.904117256370475D, -0.769902674194305D, -0.587317954286617D, -0.367831498998180D,
		 -0.125233408511469D,  0.125233408511469D,  0.367831498998180D,  0.587317954286617D,  0.769902674194305D,
		  0.904117256370475D,  0.981560634246719D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,
		  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D},
		{-0.984183054718588D, -0.917598399222978D, -0.801578090733310D, -0.642349339440340D, -0.448492751036447D,
		 -0.230458315955135D,  0.000000000000000D,  0.230458315955135D,  0.448492751036447D,  0.642349339440340D,
		  0.801578090733310D,  0.917598399222978D,  0.984183054718588D,  0.000000000000000D,  0.000000000000000D,
		  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D},
		{-0.986283808696812D, -0.928434883663574D, -0.827201315069765D, -0.687292904811685D, -0.515248636358154D,
		 -0.319112368927890D, -0.108054948707344D,  0.108054948707344D,  0.319112368927890D,  0.515248636358154D,
		  0.687292904811685D,  0.827201315069765D,  0.928434883663574D,  0.986283808696812D,  0.000000000000000D,
		  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D},
		{-0.987992518020485D, -0.937273392400706D, -0.848206583410427D, -0.724417731360170D, -0.570972172608539D,
		 -0.394151347077563D, -0.201194093997435D,  0.000000000000000D,  0.201194093997435D,  0.394151347077563D,
		  0.570972172608539D,  0.724417731360170D,  0.848206583410427D,  0.937273392400706D,  0.987992518020485D,
		  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D},
		{-0.989400934991650D, -0.944575023073233D, -0.865631202387832D, -0.755404408355003D, -0.617876244402644D,
		 -0.458016777657227D, -0.281603550779259D, -0.095012509837637D,  0.095012509837637D,  0.281603550779259D,
		  0.458016777657227D,  0.617876244402644D,  0.755404408355003D,  0.865631202387832D,  0.944575023073233D,
		  0.989400934991650D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D},
		{-0.990575475314417D, -0.950675521768768D, -0.880239153726986D, -0.781514003896801D, -0.657671159216691D,
		 -0.512690537086477D, -0.351231763453876D, -0.178484181495848D,  0.000000000000000D,  0.178484181495848D,
		  0.351231763453876D,  0.512690537086477D,  0.657671159216691D,  0.781514003896801D,  0.880239153726986D,
		  0.950675521768768D,  0.990575475314417D,  0.000000000000000D,  0.000000000000000D,  0.000000000000000D},
		{-0.991565168420931D, -0.955823949571398D, -0.892602466497556D, -0.803704958972523D, -0.691687043060353D,
		 -0.559770831073948D, -0.411751161462843D, -0.251886225691505D, -0.084775013041735D,  0.084775013041735D,
		  0.251886225691505D,  0.411751161462843D,  0.559770831073948D,  0.691687043060353D,  0.803704958972523D,
		  0.892602466497556D,  0.955823949571398D,  0.991565168420931D,  0.000000000000000D,  0.000000000000000D},
		{-0.992406843843584D, -0.960208152134830D, -0.903155903614818D, -0.822714656537143D, -0.720966177335229D,
		 -0.600545304661681D, -0.464570741375961D, -0.316564099963630D, -0.160358645640225D,  0.000000000000000D,
		  0.160358645640225D,  0.316564099963630D,  0.464570741375961D,  0.600545304661681D,  0.720966177335229D,
		  0.822714656537143D,  0.903155903614818D,  0.960208152134830D,  0.992406843843584D,  0.000000000000000D},
		{-0.993128599185095D, -0.963971927277914D, -0.912234428251326D, -0.839116971822219D, -0.746331906460151D,
		 -0.636053680726515D, -0.510867001950827D, -0.373706088715420D, -0.227785851141645D, -0.076526521133497D,
		  0.076526521133497D,  0.227785851141645D,  0.373706088715420D,  0.510867001950827D,  0.636053680726515D,
		  0.746331906460151D,  0.839116971822219D,  0.912234428251326D,  0.963971927277914D,  0.993128599185095D}
	};
	private static final double weights[][] = {
		{0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D,
		 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D,
		 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D,
		 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D},
		{2.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D,
		 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D,
		 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D,
		 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D},
		{1.000000000000000D, 1.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D,
		 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D,
		 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D,
		 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D},
		{0.555555555555556D, 0.888888888888889D, 0.555555555555556D, 0.000000000000000D, 0.000000000000000D,
		 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D,
		 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D,
		 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D},
		{0.347854845137454D, 0.652145154862546D, 0.652145154862546D, 0.347854845137454D, 0.000000000000000D,
		 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D,
		 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D,
		 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D},
		{0.236926885056189D, 0.478628670499366D, 0.568888888888889D, 0.478628670499366D, 0.236926885056189D,
		 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D,
		 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D,
		 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D},
		{0.171324492379170D, 0.360761573048139D, 0.467913934572691D, 0.467913934572691D, 0.360761573048139D,
		 0.171324492379170D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D,
		 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D,
		 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D},
		{0.129484966168870D, 0.279705391489277D, 0.381830050505119D, 0.417959183673469D, 0.381830050505119D,
		 0.279705391489277D, 0.129484966168870D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D,
		 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D,
		 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D},
		{0.101228536290376D, 0.222381034453374D, 0.313706645877887D, 0.362683783378362D, 0.362683783378362D,
		 0.313706645877887D, 0.222381034453374D, 0.101228536290376D, 0.000000000000000D, 0.000000000000000D,
		 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D,
		 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D},
		{0.081274388361574D, 0.180648160694857D, 0.260610696402935D, 0.312347077040003D, 0.330239355001260D,
		 0.312347077040003D, 0.260610696402935D, 0.180648160694857D, 0.081274388361574D, 0.000000000000000D,
		 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D,
		 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D},
		{0.066671344308688D, 0.149451349150581D, 0.219086362515982D, 0.269266719309996D, 0.295524224714753D,
		 0.295524224714753D, 0.269266719309996D, 0.219086362515982D, 0.149451349150581D, 0.066671344308688D,
		 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D,
		 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D},
		{0.055668567116174D, 0.125580369464905D, 0.186290210927734D, 0.233193764591990D, 0.262804544510247D,
		 0.272925086777901D, 0.262804544510247D, 0.233193764591990D, 0.186290210927734D, 0.125580369464905D,
		 0.055668567116174D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D,
		 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D},
		{0.047175336386512D, 0.106939325995318D, 0.160078328543346D, 0.203167426723066D, 0.233492536538355D,
		 0.249147045813403D, 0.249147045813403D, 0.233492536538355D, 0.203167426723066D, 0.160078328543346D,
		 0.106939325995318D, 0.047175336386512D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D,
		 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D},
		{0.040484004765316D, 0.092121499837728D, 0.138873510219787D, 0.178145980761946D, 0.207816047536889D,
		 0.226283180262897D, 0.232551553230874D, 0.226283180262897D, 0.207816047536889D, 0.178145980761946D,
		 0.138873510219787D, 0.092121499837728D, 0.040484004765316D, 0.000000000000000D, 0.000000000000000D,
		 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D},
		{0.035119460331752D, 0.080158087159760D, 0.121518570687903D, 0.157203167158194D, 0.185538397477938D,
		 0.205198463721296D, 0.215263853463158D, 0.215263853463158D, 0.205198463721296D, 0.185538397477938D,
		 0.157203167158194D, 0.121518570687903D, 0.080158087159760D, 0.035119460331752D, 0.000000000000000D,
		 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D},
		{0.030753241996117D, 0.070366047488108D, 0.107159220467172D, 0.139570677926154D, 0.166269205816994D,
		 0.186161000015562D, 0.198431485327112D, 0.202578241925561D, 0.198431485327112D, 0.186161000015562D,
		 0.166269205816994D, 0.139570677926154D, 0.107159220467172D, 0.070366047488108D, 0.030753241996117D,
		 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D},
		{0.027152459411754D, 0.062253523938648D, 0.095158511682493D, 0.124628971255534D, 0.149595988816577D,
		 0.169156519395003D, 0.182603415044924D, 0.189450610455069D, 0.189450610455069D, 0.182603415044924D,
		 0.169156519395003D, 0.149595988816577D, 0.124628971255534D, 0.095158511682493D, 0.062253523938648D,
		 0.027152459411754D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D},
		{0.024148302868548D, 0.055459529373987D, 0.085036148317179D, 0.111883847193404D, 0.135136368468525D,
		 0.154045761076810D, 0.168004102156450D, 0.176562705366993D, 0.179446470356207D, 0.176562705366993D,
		 0.168004102156450D, 0.154045761076810D, 0.135136368468525D, 0.111883847193404D, 0.085036148317179D,
		 0.055459529373987D, 0.024148302868548D, 0.000000000000000D, 0.000000000000000D, 0.000000000000000D},
		{0.021616013526483D, 0.049714548894970D, 0.076425730254889D, 0.100942044106287D, 0.122555206711478D,
		 0.140642914670651D, 0.154684675126265D, 0.164276483745833D, 0.169142382963144D, 0.169142382963144D,
		 0.164276483745833D, 0.154684675126265D, 0.140642914670651D, 0.122555206711478D, 0.100942044106287D,
		 0.076425730254889D, 0.049714548894970D, 0.021616013526483D, 0.000000000000000D, 0.000000000000000D},
		{0.019461788229726D, 0.044814226765700D, 0.069044542737641D, 0.091490021622450D, 0.111566645547334D,
		 0.128753962539336D, 0.142606702173607D, 0.152766042065860D, 0.158968843393954D, 0.161054449848784D,
		 0.158968843393954D, 0.152766042065860D, 0.142606702173607D, 0.128753962539336D, 0.111566645547334D,
		 0.091490021622450D, 0.069044542737641D, 0.044814226765700D, 0.019461788229726D, 0.000000000000000D},
		{0.017614007139152D, 0.040601429800387D, 0.062672048334109D, 0.083276741576705D, 0.101930119817240D,
		 0.118194531961518D, 0.131688638449177D, 0.142096109318382D, 0.149172986472604D, 0.152753387130726D,
		 0.152753387130726D, 0.149172986472604D, 0.142096109318382D, 0.131688638449177D, 0.118194531961518D,
		 0.101930119817240D, 0.083276741576705D, 0.062672048334109D, 0.040601429800387D, 0.017614007139152D}
	};
	
	double relativeAccuracy;
	double absoluteAccuracy;
	int minIterations;
	int maxIterations;
	
	public GaussLagrange(double relativeAccuracy, double absoluteAccuracy, int minIterations, int maxIterations) {
		this.relativeAccuracy = relativeAccuracy;
		this.absoluteAccuracy = absoluteAccuracy;
		this.minIterations = Math.max(1, Math.min(minIterations, 30));
		this.maxIterations = Math.min(30, Math.max(maxIterations, 1));
	}
	
	public double integrate(int maxIterations, UnivariateFunction f, double a, double b) {
		if (a == b) {
			return 0.0; // No Dirac delta functions please!
		}
		int N = Math.min(maxIterations, this.maxIterations);
		double w = 0;
		double z = 0;
		double I = 0;
		double I_prev = 0;
		for (int n = minIterations; n < N; n++) {
			I = 0;
			for (int i = 0; i < n; i++) {
				z = 0.5 * (b - a) * points[n][i] + 0.5 * (b + a);
				w = 0.5 * (b - a) * weights[n][i];
				I += w * f.value(z);
			}
			double absError = Math.abs(I-I_prev);
			double relError = relativeAccuracy;
			if (I!= 0.0){
				relError = absError/Math.abs(I);
			}
			if (absError < absoluteAccuracy || relError < relativeAccuracy)
			{
				return I;
			}
			I_prev = I;
		}
		return 0.0;
	}

}
