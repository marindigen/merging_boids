package boids

import munit.Assertions.*
import cs214.*
import scala.annotation.targetName

val DELTA = 0.005f

def assertEqualsVector2(obtained: Vector2, expected: Vector2)(using munit.Location) =
  assertEqualsFloat(obtained.x, expected.x, DELTA)
  assertEqualsFloat(obtained.y, expected.y, DELTA)

def assertEqualsBoid(obtained: Boid, expected: Boid)(using munit.Location) =
  assert(
    equalsBoid(obtained, expected),
    f"expected boid $expected, obtained $obtained"
  )

def equalsFloat(f1: Float, f2: Float): Boolean =
  val exactlyTheSame = java.lang.Float.compare(f1, f2) == 0
  val almostTheSame = Math.abs(f1 - f2) <= DELTA
  exactlyTheSame || almostTheSame

def equalsBoid(b1: Boid, b2: Boid): Boolean =
  equalsFloat(b1.position.x, b2.position.x) &&
    equalsFloat(b1.position.y, b2.position.y) &&
    equalsFloat(b1.velocity.x, b2.velocity.x) &&
    equalsFloat(b1.velocity.y, b2.velocity.y)

def boidSequenceToSeqBoid(bs: BoidSequence): Seq[Boid] =
  Seq.unfold(bs)(s => Option.unless(s.isEmpty)((s.head, s.tail)))

def seqBoidToBoidSequence(sb: Boid*): BoidSequence =
  seqBoidToBoidSequence(sb)

@targetName("seqBoidToBoidSequenceStar")
def seqBoidToBoidSequence(sb: Seq[Boid]) =
  sb.foldRight[BoidSequence](BoidNil())((b, bs) => BoidCons(b, bs))

def assertEqualsSeqBoid(obtained: Seq[Boid], expected: Seq[Boid], physics: Physics)(using munit.Location) =
  for boid <- obtained do
    assert(
      physics.minimumSpeed - DELTA <= boid.velocity.norm
        && physics.maximumSpeed + DELTA >= boid.velocity.norm,
      f"boid sequence contains a boid with velocity out of bounds: ${boid.velocity}"
    )
    assert(
      boid.position.x.isFinite && boid.position.y.isFinite
        && boid.velocity.x.isFinite && boid.velocity.y.isFinite,
      f"boid sequence contains a boid with nonfinite parameters: $boid"
    )
  assert(
    obtained.forall(bo => expected.exists(be => equalsBoid(bo, be))) &&
      expected.forall(be => obtained.exists(bo => equalsBoid(bo, be))) &&
      obtained.length == expected.length,
    f"obtained boid sequence $obtained does not match expected $expected"
  )

def assertEqualsBoidSequence(obtained: BoidSequence, expected: BoidSequence, physics: Physics)(using munit.Location) =
  assertEqualsSeqBoid(boidSequenceToSeqBoid(obtained), boidSequenceToSeqBoid(expected), physics)

def readTestCase(name: String)(using munit.Location) =
  val relPath = os.rel / "src" / "test" / "json" / (name + ".json")
  val testCaseRaw =
    try
      os.read(os.pwd / relPath)
    catch
      case _: java.nio.file.NoSuchFileException =>
        os.read(os.pwd / "labs" / "boids" / relPath)
  ujson.read(testCaseRaw)

def jsonToBoidSequence(boids: ujson.Value): BoidSequence =
  seqBoidToBoidSequence(
    boids.arr.map(b => Boid(Vector2(b("x").num, b("y").num), Vector2(b("vx").num, b("vy").num))).toSeq
  )

def jsonToPhysics(physics: ujson.Value): Physics =
  upickle.default.read[Physics](physics)

def runTestCase(name: String)(using munit.Location) =
  val testCase = readTestCase(name)
  val initBoids = jsonToBoidSequence(testCase("initialBoids"))
  val physics = jsonToPhysics(testCase("physics"))
  testCase("reference")
    .arr
    .map(jsonToBoidSequence)
    .toSeq
    .drop(1)
    .foldLeft(initBoids) { (world, expected) =>
      val next = tickWorld(world, physics)
      assertEqualsBoidSequence(next, expected, physics)
      next
    }

def seqBoidToJson(boids: Seq[Boid]) =
  boids.map(b =>
    ujson.Obj(
      "x" -> b.position.x,
      "y" -> b.position.y,
      "vx" -> b.velocity.x,
      "vy" -> b.velocity.y
    )
  )

given boundingBoxWriter: upickle.default.ReadWriter[BoundingBox] = upickle.default.macroRW
given physicsWriter: upickle.default.ReadWriter[Physics] = upickle.default.macroRW

def generateTestCase(name: String, initialBoids: Seq[Boid], physics: Physics, steps: Int = 50) =
  val path = os.pwd / "labs" / "boids" / "src" / "test" / "json" / (name + ".json")
  val reference = Seq.iterate(seqBoidToBoidSequence(initialBoids), steps)(boids => tickWorld(boids, physics))
  val testCase = ujson.Obj(
    "initialBoids" -> seqBoidToJson(initialBoids),
    "physics" -> upickle.default.writeJs(physics),
    "reference" -> reference.map(boidSequenceToSeqBoid).map(seqBoidToJson)
  )
  os.write.over(path, ujson.write(testCase, 4, true))

def generateTestCases() =
  val noForces = Physics(
    limits = Physics.defaultLimits,
    minimumSpeed = 2f,
    maximumSpeed = 4f,
    perceptionRadius = 80f,
    avoidanceRadius = 22f,
    // all weights are zero
    avoidanceWeight = 0f,
    cohesionWeight = 0f,
    alignmentWeight = 0f,
    containmentWeight = 0f
  )
  generateTestCase(
    "00_singleBoidNoForces",
    Seq(Boid(Vector2(1, -45), Vector2(2, 3))),
    noForces
  )
  generateTestCase(
    "01_threeBoidsNoForces",
    Seq(
      Boid(Vector2(1, -45), Vector2(2, 3)),
      Boid(Vector2(10, -45), Vector2(-2, 1)),
      Boid(Vector2(100, 49), Vector2(-2, -2))
    ),
    noForces
  )
  // val onlyAvoidance = Physics(
  //   limits = Physics.defaultLimits,
    // set min and max speed so that they won't affect the tests
  //   minimumSpeed = 0f,
  //   maximumSpeed = 100f,
  //   perceptionRadius = 80f,
    // only the avoidance parameters are relevant here
  //   avoidanceRadius = 22f,
  //   avoidanceWeight = 10f,
  //   cohesionWeight = 0f,
  //   alignmentWeight = 0f,
  //   containmentWeight = 0f
  // )
  // generateTestCase(
  //   "10_singleBoidAvoidance",
  //   Seq(Boid(Vector2(1, -45), Vector2(2, 3))),
  //   onlyAvoidance
  // )
  // generateTestCase(
  //   "11_twoBoidsAvoidanceX",
  //   Seq(Boid(Vector2(34, -45), Vector2(-2, 0)), Boid(Vector2(10, -45), Vector2(2, 0))),
  //   onlyAvoidance
  // )
  // generateTestCase(
  //   "12_twoBoidsAvoidanceXY",
  //   Seq(Boid(Vector2(34, -45), Vector2(-2, 1)), Boid(Vector2(10, -45), Vector2(2, 1))),
  //   onlyAvoidance
  // )
  // generateTestCase(
  //   "13_twoBoidsAvoidanceFar",
  //   Seq(Boid(Vector2(34, -45), Vector2(-2, 1)), Boid(Vector2(10, -145), Vector2(2, -1))),
  //   onlyAvoidance
  // )
  // generateTestCase(
  //   "14_mixedAvoidance",
  //   Seq(
  //     Boid(Vector2(34, -45), Vector2(-2, 1)),
  //     Boid(Vector2(10, -145), Vector2(2, 0)),
  //     Boid(Vector2(34, -35), Vector2(-2, 1)),
  //     Boid(Vector2(10, -160), Vector2(2, 0))
  //   ),
  //   onlyAvoidance
  // )
  // generateTestCase(
  //   "15_avoidanceSamePosition",
  //   Seq(
  //     Boid(Vector2(34, -45), Vector2(-2, 1)),
  //     Boid(Vector2(10, -145), Vector2(2, 0)),
  //     Boid(Vector2(34, -35), Vector2(-2, 1)),
  //     Boid(Vector2(10, -160), Vector2(2, 0)),
  //     Boid(Vector2(10, -160), Vector2(-2, 0))
  //   ),
  //   onlyAvoidance
  // )

  val onlyCohesion = Physics(
    limits = Physics.defaultLimits,
    // set min and max speed so that they won't affect the tests
    minimumSpeed = 0f,
    maximumSpeed = 6f,
    // avoidance and cohesion parameters are relevant
    perceptionRadius = 80f,
    avoidanceRadius = 22f,
    avoidanceWeight = 0f,
    cohesionWeight = 0.1f,
    // these two are still zero
    alignmentWeight = 0f,
    containmentWeight = 0f
  )
  generateTestCase(
    "20_twoBoidsRestCohesion",
    Seq(Boid(Vector2(40, 0), Vector2(0, 0)), Boid(Vector2(10, -40), Vector2(0, 0))),
    onlyCohesion
  )
  generateTestCase(
    "21_twoBoidsCohesionDance",
    Seq(Boid(Vector2(40, 0), Vector2(0, 0)), Boid(Vector2(10, -40), Vector2(0, 1))),
    onlyCohesion
  )
  generateTestCase(
    "22_twoBoidsCohesionFar",
    Seq(Boid(Vector2(40, -200), Vector2(0, -1)), Boid(Vector2(10, -40), Vector2(0, 1))),
    onlyCohesion
  )

  val avoidanceAndCohesion = Physics(
    limits = Physics.defaultLimits,
    // set min and max speed so that they won't affect the tests
    minimumSpeed = 0f,
    maximumSpeed = 100f,
    // avoidance and cohesion parameters are relevant
    perceptionRadius = 80f,
    avoidanceRadius = 22f,
    avoidanceWeight = 10f,
    cohesionWeight = 0.1f,
    // these two are still zero
    alignmentWeight = 0f,
    containmentWeight = 0f
  )
  generateTestCase(
    "30_avoidanceCohesion",
    Seq(Boid(Vector2(40, 0), Vector2(0, 0)), Boid(Vector2(10, -40), Vector2(0, 0))),
    avoidanceAndCohesion,
    steps = 4
  )
  generateTestCase(
    "31_avoidanceCohesionLonger",
    Seq(Boid(Vector2(40, 0), Vector2(0, 0)), Boid(Vector2(10, -40), Vector2(0, 0))),
    avoidanceAndCohesion
  )

  val threeBodyProblem = Physics(
    limits = Physics.defaultLimits,
    minimumSpeed = 0f,
    maximumSpeed = 5f,
    perceptionRadius = 200f,
    avoidanceRadius = 35f,
    avoidanceWeight = 8f,
    cohesionWeight = 0.001f,
    alignmentWeight = 0f,
    containmentWeight = 0f
  )
  generateTestCase(
    "32_threeBodyProblem",
    Seq(
      Boid(Vector2(34, -45), Vector2(-2, 1)),
      Boid(Vector2(10, -145), Vector2(2, 1)),
      Boid(Vector2(-90, -95), Vector2(2, -1))
    ),
    threeBodyProblem,
    steps = 300
  )

  val onlyAlignment = Physics(
    limits = Physics.defaultLimits,
    minimumSpeed = 0f,
    maximumSpeed = 5f,
    perceptionRadius = 200f,
    avoidanceRadius = 35f,
    avoidanceWeight = 0f,
    cohesionWeight = 0f,
    alignmentWeight = 0.04f,
    containmentWeight = 0f
  )
  generateTestCase(
    "40_onlyAlignment",
    Seq(
      Boid(Vector2(34, -45), Vector2(-1, 1)),
      Boid(Vector2(10, -145), Vector2(2, 1)),
      Boid(Vector2(-90, -95), Vector2(2, -1))
    ),
    onlyAlignment
  )
  generateTestCase(
    "41_alignmentFar",
    Seq(
      Boid(Vector2(112, -45), Vector2(2, 1)),
      Boid(Vector2(-89, -145), Vector2(1, 2)),
      Boid(Vector2(-290, -95), Vector2(0, 1.5))
    ),
    onlyAlignment
  )

  val onlyContainment = Physics(
    limits = Physics.defaultLimits,
    minimumSpeed = 0f,
    maximumSpeed = 5f,
    perceptionRadius = 200f,
    avoidanceRadius = 35f,
    avoidanceWeight = 0f,
    cohesionWeight = 0f,
    alignmentWeight = 0f,
    containmentWeight = 0.5f
  )
  generateTestCase(
    "50_containmentTop",
    Seq(Boid(Vector2(0, onlyContainment.limits.ymin + 20.0), Vector2(1, -1))),
    onlyContainment
  )
  generateTestCase(
    "51_containmentBottom",
    Seq(Boid(Vector2(0, onlyContainment.limits.ymax - 20.0), Vector2(1, 1))),
    onlyContainment
  )
  generateTestCase(
    "52_containmentLeft",
    Seq(Boid(Vector2(onlyContainment.limits.xmin + 20.0, -50), Vector2(-1, -1))),
    onlyContainment
  )
  generateTestCase(
    "53_containmentRight",
    Seq(Boid(Vector2(onlyContainment.limits.xmax - 20.0, -50), Vector2(1, -1))),
    onlyContainment
  )
  generateTestCase(
    "54_containmentCumulative",
    Seq(Boid(Vector2(onlyContainment.limits.xmin + 30.0, onlyContainment.limits.ymin + 20.0), Vector2(-1, -1))),
    onlyContainment
  )

  val allTogether = Physics(
    limits = Physics.defaultLimits,
    minimumSpeed = 2f,
    maximumSpeed = 5f,
    perceptionRadius = 80f,
    avoidanceRadius = 15f,
    avoidanceWeight = 1f,
    cohesionWeight = 0.001f,
    alignmentWeight = 0.027f,
    containmentWeight = 0.5f
  )
  generateTestCase(
    "60_allTogether",
    Seq(
      Boid(Vector2(34, -45), Vector2(-2, 1)),
      Boid(Vector2(10, -145), Vector2(2, 1)),
      Boid(Vector2(-90, -95), Vector2(2, -1)),
      Boid(Vector2(74, -275), Vector2(2, 1)),
      Boid(Vector2(-380, -225), Vector2(2, -1)),
      Boid(Vector2(-11, -75), Vector2(2, 1)),
      Boid(Vector2(74, -265), Vector2(-2, -1)),
      Boid(Vector2(-380, -215), Vector2(2, 1)),
      Boid(Vector2(-28, -75), Vector2(2, -1))
    ),
    allTogether,
    steps = 200
  )

  val chaoticCohesion = Physics(
    limits = BoundingBox(0, 1000, 0, 700),
    minimumSpeed = 3f,
    maximumSpeed = 5f,
    perceptionRadius = 100f,
    avoidanceRadius = 22f,
    avoidanceWeight = 0f,
    cohesionWeight = 0.0085f,
    alignmentWeight = 0f,
    containmentWeight = 0f
  )
  generateTestCase(
    "23_chaoticCohesion",
    Seq(
      Boid(Vector2(977.22925, 672.0462), Vector2(2.0969963, 6.2409096)),
      Boid(Vector2(209.34862, 624.7859), Vector2(2.9390798, 2.9082901)),
      Boid(Vector2(930.4992, 367.7411), Vector2(7.261567, 2.3651261)),
      Boid(Vector2(635.08734, 471.325), Vector2(-0.48436433, 1.0807203)),
      Boid(Vector2(445.3669, 611.80225), Vector2(6.081317, -3.9573388)),
      Boid(Vector2(237.25034, 63.008785), Vector2(-4.4421453, -5.4488406)),
      Boid(Vector2(490.40778, 166.33987), Vector2(-1.6041406, -0.5737326)),
      Boid(Vector2(271.84946, 34.961246), Vector2(-2.5027463, 0.040716484)),
      Boid(Vector2(396.91125, 348.12128), Vector2(-4.9359026, 1.0932642)),
      Boid(Vector2(81.35802, 625.39307), Vector2(0.6653953, 7.515667)),
      Boid(Vector2(386.19168, 322.14893), Vector2(-4.7752333, -0.91836536)),
      Boid(Vector2(623.836, 400.99933), Vector2(-1.1569461, -4.7367315)),
      Boid(Vector2(129.92387, 689.6881), Vector2(0.64536846, -2.82881)),
      Boid(Vector2(476.31818, 625.32855), Vector2(0.13388996, -7.835241)),
      Boid(Vector2(203.76343, 585.1451), Vector2(-0.0027386067, -1.7927363)),
      Boid(Vector2(460.32346, 553.22485), Vector2(-3.6060457, -1.9197826)),
      Boid(Vector2(846.26965, 545.3188), Vector2(-5.6607943, 4.7123647)),
      Boid(Vector2(167.22536, 194.50441), Vector2(-4.2864046, 6.024211)),
      Boid(Vector2(393.15683, 260.54355), Vector2(3.905466, -6.3205557)),
      Boid(Vector2(697.357, 96.96125), Vector2(-2.3646262, 5.0268936)),
      Boid(Vector2(121.15723, 60.113274), Vector2(-0.8907776, -6.074618)),
      Boid(Vector2(869.1072, 452.6049), Vector2(-2.4155512, -0.3129508)),
      Boid(Vector2(6.2866807, 624.1221), Vector2(-5.9914484, 4.275636)),
      Boid(Vector2(2.8957725, 431.71698), Vector2(-6.3573174, 0.6957431)),
      Boid(Vector2(78.06295, 150.47862), Vector2(-0.8271307, 1.1409128)),
      Boid(Vector2(139.80067, 596.8725), Vector2(-1.6421447, 3.3858523)),
      Boid(Vector2(634.23206, 139.34607), Vector2(4.713855, -4.026327)),
      Boid(Vector2(215.74039, 443.50406), Vector2(-1.8553545, 1.9486046)),
      Boid(Vector2(569.8192, 387.63284), Vector2(-4.209829, -0.35940158)),
      Boid(Vector2(279.5005, 154.89716), Vector2(-3.6094003, -5.512343)),
      Boid(Vector2(469.8251, 613.40295), Vector2(-4.439414, 3.9073174)),
      Boid(Vector2(316.52277, 347.68765), Vector2(-0.33629593, -5.435314)),
      Boid(Vector2(916.64886, 154.1979), Vector2(1.6666675, 0.39559606)),
      Boid(Vector2(620.26135, 518.9909), Vector2(0.5892067, -1.5210886)),
      Boid(Vector2(671.3261, 604.2566), Vector2(1.5764387, -3.4759192)),
      Boid(Vector2(939.0408, 188.22623), Vector2(-1.3304471, -0.0045436537)),
      Boid(Vector2(1.9264221, 473.49216), Vector2(-1.7082727, -0.08067199)),
      Boid(Vector2(818.73486, 55.508156), Vector2(-5.773322, -2.9640784)),
      Boid(Vector2(452.96646, 165.82973), Vector2(0.22411452, 3.6757572)),
      Boid(Vector2(660.21814, 674.1611), Vector2(-5.9580984, 1.4528364)),
      Boid(Vector2(776.44763, 586.1307), Vector2(1.1824785, 1.820576)),
      Boid(Vector2(15.42288, 357.46988), Vector2(-2.7194266, -6.5442877)),
      Boid(Vector2(578.14014, 593.42487), Vector2(2.442421, 6.786648)),
      Boid(Vector2(556.9709, 209.2156), Vector2(-2.6419618, 2.3993702)),
      Boid(Vector2(471.98868, 323.54373), Vector2(7.424833, 0.21514644)),
      Boid(Vector2(973.72754, 83.85193), Vector2(-1.1830224, 1.2189468)),
      Boid(Vector2(181.02586, 506.4925), Vector2(-1.9136099, 3.2894366)),
      Boid(Vector2(768.26227, 345.36234), Vector2(2.7351923, -4.0121555)),
      Boid(Vector2(74.93615, 603.02203), Vector2(4.7042165, 2.1012533)),
      Boid(Vector2(562.0808, 603.8966), Vector2(0.64281845, 5.819615)),
      Boid(Vector2(92.42016, 280.8557), Vector2(0.5689111, -6.325644)),
      Boid(Vector2(50.62652, 566.2185), Vector2(-0.5900483, -4.153025)),
      Boid(Vector2(860.7689, 143.89857), Vector2(0.15528722, -1.5911096)),
      Boid(Vector2(192.4333, 239.80626), Vector2(1.6156538, -3.8054798)),
      Boid(Vector2(95.563416, 417.5017), Vector2(3.4328928, -6.684673)),
      Boid(Vector2(815.363, 339.37772), Vector2(7.1493654, 0.05618316)),
      Boid(Vector2(744.56177, 513.1192), Vector2(4.700848, -1.5433841)),
      Boid(Vector2(542.9177, 262.54083), Vector2(2.9489229, -2.2477148)),
      Boid(Vector2(656.74915, 615.0173), Vector2(1.459987, -2.0292864)),
      Boid(Vector2(362.35434, 248.66423), Vector2(2.8928094, -0.47656435)),
      Boid(Vector2(860.7467, 301.75394), Vector2(-1.3928497, 7.3479853)),
      Boid(Vector2(40.679993, 277.00696), Vector2(-0.86613476, 7.3941426)),
      Boid(Vector2(215.3331, 282.0296), Vector2(2.3141527, -3.0580993)),
      Boid(Vector2(4.0676, 515.79944), Vector2(4.3430443, -4.9588614)),
      Boid(Vector2(514.2129, 122.55767), Vector2(-3.1054406, -2.2874255)),
      Boid(Vector2(380.6231, 462.85413), Vector2(-1.2555326, 3.2968788)),
      Boid(Vector2(922.6729, 181.87962), Vector2(4.9568563, 3.652516)),
      Boid(Vector2(387.42566, 258.5952), Vector2(-5.262066, -0.23592007)),
      Boid(Vector2(750.04266, 427.47418), Vector2(-4.397234, -0.6054327)),
      Boid(Vector2(952.3756, 445.7713), Vector2(-6.1632524, 4.8590097)),
      Boid(Vector2(470.46674, 211.93817), Vector2(-4.803925, 2.7399263)),
      Boid(Vector2(231.88985, 289.8629), Vector2(4.953639, 0.61127657)),
      Boid(Vector2(753.1261, 284.70114), Vector2(-1.43835, -5.2035203)),
      Boid(Vector2(862.2452, 210.81465), Vector2(2.9362862, -5.8990655)),
      Boid(Vector2(17.932117, 463.29474), Vector2(2.2251003, 0.49118105)),
      Boid(Vector2(365.37582, 610.22205), Vector2(-1.0286889, -0.1258832)),
      Boid(Vector2(693.84595, 82.62952), Vector2(-3.2542117, -3.4227436)),
      Boid(Vector2(119.1895, 108.46981), Vector2(-5.1737328, 2.5892334)),
      Boid(Vector2(924.3622, 182.04297), Vector2(2.5979004, -3.3510783)),
      Boid(Vector2(697.9754, 133.56844), Vector2(3.8886452, 2.5448303)),
      Boid(Vector2(938.3258, 178.06445), Vector2(4.5199337, -4.376108)),
      Boid(Vector2(190.23358, 327.55365), Vector2(-0.04482969, -5.154569)),
      Boid(Vector2(684.1359, 605.9015), Vector2(-0.035936464, 1.7158024)),
      Boid(Vector2(309.26443, 157.31197), Vector2(-5.7490134, 1.2016836)),
      Boid(Vector2(611.5223, 194.5864), Vector2(1.3345199, -3.9840727)),
      Boid(Vector2(726.11383, 34.565086), Vector2(6.232904, -4.6872516)),
      Boid(Vector2(677.46265, 369.52994), Vector2(1.9350342, -4.7669644)),
      Boid(Vector2(74.97823, 664.49274), Vector2(4.657794, 3.7299922)),
      Boid(Vector2(751.44275, 351.91205), Vector2(-4.235987, 0.046551887)),
      Boid(Vector2(507.9304, 54.09428), Vector2(-2.7557847, 3.0428019)),
      Boid(Vector2(600.14307, 346.89975), Vector2(-4.9599566, -5.3212996)),
      Boid(Vector2(508.83627, 419.88565), Vector2(-1.2000545, -2.269056)),
      Boid(Vector2(542.7696, 82.11411), Vector2(-0.99833477, 0.5186262)),
      Boid(Vector2(915.5892, 539.8995), Vector2(-2.7311182, 1.1090834)),
      Boid(Vector2(685.62775, 674.0781), Vector2(4.7029195, 4.291865)),
      Boid(Vector2(745.2969, 97.20183), Vector2(1.5077772, 0.59469086)),
      Boid(Vector2(368.17313, 563.56006), Vector2(4.4845414, 5.3834224)),
      Boid(Vector2(468.81448, 661.2608), Vector2(2.9881625, -0.42305169)),
      Boid(Vector2(82.65376, 170.78587), Vector2(-3.5700386, 3.0642612)),
      Boid(Vector2(839.02325, 149.49738), Vector2(-5.736677, 0.6759297)),
      Boid(Vector2(665.5927, 478.42825), Vector2(4.1566806, -0.83647096)),
      Boid(Vector2(241.51414, 504.22104), Vector2(-0.38489547, -1.8444356)),
      Boid(Vector2(1.7415881, 154.08717), Vector2(-1.567473, -3.0536947)),
      Boid(Vector2(806.8424, 9.898048), Vector2(-1.6585073, 7.5115385)),
      Boid(Vector2(829.7693, 374.6619), Vector2(-4.820791, -2.5829108)),
      Boid(Vector2(200.2772, 662.32166), Vector2(-1.2994112, 1.6664432)),
      Boid(Vector2(493.8329, 112.57201), Vector2(-2.0934284, 1.3804953)),
      Boid(Vector2(87.97789, 484.74945), Vector2(-4.425876, 0.7768286)),
      Boid(Vector2(646.3092, 388.2191), Vector2(0.4920658, -6.57405)),
      Boid(Vector2(767.787, 66.44449), Vector2(3.3770063, 1.0845509)),
      Boid(Vector2(974.351, 184.38907), Vector2(1.0322762, -2.0437164)),
      Boid(Vector2(348.33752, 176.30223), Vector2(1.592914, 1.6986022)),
      Boid(Vector2(996.56116, 56.479053), Vector2(4.6126094, -2.0317183)),
      Boid(Vector2(161.92924, 478.36768), Vector2(4.811825, 5.8475986)),
      Boid(Vector2(855.9646, 357.49023), Vector2(-1.3508729, 4.8593416)),
      Boid(Vector2(804.63794, 47.04138), Vector2(0.55934125, 2.693421)),
      Boid(Vector2(821.58716, 28.664), Vector2(4.1500397, 3.6274257)),
      Boid(Vector2(311.37793, 347.64594), Vector2(1.8827991, 4.6089473)),
      Boid(Vector2(903.3364, 485.4443), Vector2(-0.544333, -2.3236835)),
      Boid(Vector2(494.14676, 312.76947), Vector2(-5.779692, 2.7060802)),
      Boid(Vector2(583.50946, 54.135418), Vector2(-6.125575, 3.5406258)),
      Boid(Vector2(398.43887, 106.42396), Vector2(-2.7609286, -3.8410378)),
      Boid(Vector2(173.1382, 487.18088), Vector2(-3.5350957, -5.398712)),
      Boid(Vector2(165.8228, 626.5528), Vector2(0.90134114, 0.58769596)),
      Boid(Vector2(712.5135, 404.4246), Vector2(-0.86855906, 2.1085742)),
      Boid(Vector2(329.9331, 551.7925), Vector2(0.41443166, -1.7146255)),
      Boid(Vector2(135.6305, 536.4254), Vector2(-1.332181, -1.1540045)),
      Boid(Vector2(548.44354, 486.34317), Vector2(-0.19822244, -6.2088375)),
      Boid(Vector2(601.7239, 374.02692), Vector2(5.811062, 1.2535826)),
      Boid(Vector2(599.7555, 655.79364), Vector2(-2.817515, 0.06561069)),
      Boid(Vector2(335.57474, 502.25076), Vector2(1.6960363, -7.368873)),
      Boid(Vector2(507.5394, 349.31342), Vector2(1.278936, 3.4416459)),
      Boid(Vector2(796.8182, 55.669792), Vector2(7.14156, -1.5849485)),
      Boid(Vector2(781.75775, 380.08948), Vector2(-1.8793075, 1.9312807)),
      Boid(Vector2(414.4826, 337.68018), Vector2(3.7263615, 2.6233923)),
      Boid(Vector2(448.0639, 284.68143), Vector2(-2.958469, 3.8493555)),
      Boid(Vector2(218.42343, 22.003006), Vector2(-2.987449, 4.162401)),
      Boid(Vector2(371.18994, 214.78879), Vector2(-4.574407, -0.19825108)),
      Boid(Vector2(125.516655, 401.04993), Vector2(-3.4936326, -6.162418)),
      Boid(Vector2(208.02426, 370.05792), Vector2(-1.2081076, -1.4161628)),
      Boid(Vector2(722.3166, 629.8302), Vector2(4.6810036, 0.17592816)),
      Boid(Vector2(572.8057, 662.7446), Vector2(-1.1091857, -2.072455)),
      Boid(Vector2(93.89174, 485.07486), Vector2(-2.2397857, -1.7085149)),
      Boid(Vector2(782.99585, 636.9849), Vector2(3.0859482, -4.808444)),
      Boid(Vector2(362.80972, 127.12349), Vector2(1.7936552, -3.698336)),
      Boid(Vector2(246.77367, 344.23203), Vector2(0.72410023, -2.7555084)),
      Boid(Vector2(122.74927, 516.86487), Vector2(1.2390679, 1.8398398)),
      Boid(Vector2(151.72655, 602.10315), Vector2(-3.7263348, 3.2887545)),
      Boid(Vector2(877.97687, 101.6456), Vector2(4.357087, -4.253689)),
      Boid(Vector2(619.2004, 36.438), Vector2(-4.2381287, 0.5205912)),
      Boid(Vector2(932.93146, 42.55463), Vector2(0.63088477, 2.8477793)),
      Boid(Vector2(899.8976, 525.8036), Vector2(-4.4002204, -5.1752667)),
      Boid(Vector2(915.4203, 210.86456), Vector2(3.1285427, 2.667977)),
      Boid(Vector2(503.70013, 533.98987), Vector2(-1.2217413, -0.26583403)),
      Boid(Vector2(262.43503, 633.6559), Vector2(-3.8125477, 4.626673)),
      Boid(Vector2(380.32346, 529.22284), Vector2(2.1806102, -3.194598)),
      Boid(Vector2(549.3571, 417.7707), Vector2(-2.6857846, 2.952475)),
      Boid(Vector2(397.2937, 348.88193), Vector2(-7.9031157, 0.40796256)),
      Boid(Vector2(501.3265, 127.24115), Vector2(4.7845106, -3.4343104)),
      Boid(Vector2(397.8696, 175.44089), Vector2(-5.861468, 2.3924022)),
      Boid(Vector2(889.8087, 432.20468), Vector2(2.2059608, -2.406091)),
      Boid(Vector2(63.916683, 143.15982), Vector2(-2.8693755, -7.365621)),
      Boid(Vector2(287.90033, 687.0691), Vector2(-2.031584, -2.6938884)),
      Boid(Vector2(657.37164, 595.7416), Vector2(-0.53935856, -6.5837083)),
      Boid(Vector2(834.67285, 232.3071), Vector2(-6.810284, 0.01680523)),
      Boid(Vector2(468.8193, 662.7214), Vector2(-1.1091719, 0.43462646)),
      Boid(Vector2(741.8609, 347.7599), Vector2(-0.91397476, 1.5984615)),
      Boid(Vector2(462.83615, 216.79443), Vector2(-2.4795694, 5.244797)),
      Boid(Vector2(265.46066, 115.69741), Vector2(-4.5074654, 1.6024567)),
      Boid(Vector2(177.13828, 15.5943985), Vector2(0.8738889, 4.0655756)),
      Boid(Vector2(497.5963, 192.9795), Vector2(7.2027106, 0.3284906)),
      Boid(Vector2(824.57007, 64.01164), Vector2(-3.567925, -1.1110774)),
      Boid(Vector2(13.028741, 520.8319), Vector2(7.91998, -0.63165164)),
      Boid(Vector2(61.391235, 92.208305), Vector2(-7.6453376, -1.6824105)),
      Boid(Vector2(700.0793, 46.07774), Vector2(-2.9229965, -0.40093723)),
      Boid(Vector2(414.25223, 632.8589), Vector2(2.3377533, -0.2400136)),
      Boid(Vector2(699.268, 601.9507), Vector2(-4.904653, -5.61769)),
      Boid(Vector2(954.21375, 29.61725), Vector2(1.1430587, 3.1836545)),
      Boid(Vector2(998.94775, 677.1615), Vector2(-3.4048624, -7.075922)),
      Boid(Vector2(592.7362, 673.2337), Vector2(-1.572626, -0.7681878)),
      Boid(Vector2(878.5698, 369.80994), Vector2(2.6361363, 1.9945945)),
      Boid(Vector2(830.24615, 663.56256), Vector2(1.2145083, -0.75181836)),
      Boid(Vector2(371.08563, 388.40375), Vector2(5.6826553, -0.25901836)),
      Boid(Vector2(98.135826, 552.60004), Vector2(-1.0989426, -7.690897)),
      Boid(Vector2(730.1928, 415.1072), Vector2(0.83126235, -1.0137227)),
      Boid(Vector2(496.98734, 480.74072), Vector2(1.4856323, 1.8316151)),
      Boid(Vector2(490.30728, 353.11636), Vector2(-2.545216, -0.23123825)),
      Boid(Vector2(948.04407, 521.789), Vector2(7.3674803, -0.333939)),
      Boid(Vector2(809.66046, 465.42038), Vector2(3.6429965, -4.3482766)),
      Boid(Vector2(15.020251, 576.80896), Vector2(-4.3907695, 3.345286)),
      Boid(Vector2(594.651, 244.83429), Vector2(-0.4006519, -2.9455268)),
      Boid(Vector2(873.20844, 60.366787), Vector2(2.5179536, -3.3231685)),
      Boid(Vector2(162.1961, 631.4635), Vector2(-3.0249054, 0.46078965)),
      Boid(Vector2(313.97717, 642.14825), Vector2(-1.5386056, -2.9277332)),
      Boid(Vector2(936.08374, 249.12886), Vector2(1.1266834, 4.233409)),
      Boid(Vector2(990.13275, 673.2471), Vector2(-2.5461614, 7.4978333)),
      Boid(Vector2(818.8363, 384.5297), Vector2(0.30256608, 2.7112644)),
      Boid(Vector2(331.25418, 639.6007), Vector2(3.3788896, 3.495915)),
      Boid(Vector2(432.35385, 282.04153), Vector2(2.485998, -6.982074)),
      Boid(Vector2(687.9908, 245.10971), Vector2(-2.7601302, 0.55908483)),
      Boid(Vector2(378.61157, 673.0421), Vector2(-0.7820291, 1.4038513)),
      Boid(Vector2(300.4455, 233.91002), Vector2(0.13354957, -6.651861)),
      Boid(Vector2(35.94583, 1.5298247), Vector2(4.904755, -3.2958198)),
      Boid(Vector2(628.1511, 515.0773), Vector2(-2.6429594, 3.7786374)),
      Boid(Vector2(176.96983, 347.5463), Vector2(-1.3428849, 0.049541656)),
      Boid(Vector2(427.7776, 153.72356), Vector2(1.6365302, 0.85243165)),
      Boid(Vector2(870.9178, 686.53326), Vector2(5.2608514, 4.69423)),
      Boid(Vector2(719.5919, 440.0084), Vector2(-1.1127822, 5.257739)),
      Boid(Vector2(716.2244, 656.25305), Vector2(1.9519818, 1.593446)),
      Boid(Vector2(444.82584, 42.016773), Vector2(1.8600743, 2.4781153)),
      Boid(Vector2(438.4047, 433.2703), Vector2(6.5952373, -3.8736749)),
      Boid(Vector2(183.4885, 189.76753), Vector2(4.493277, 3.0581207)),
      Boid(Vector2(283.80646, 248.24107), Vector2(0.6879786, -0.7293234)),
      Boid(Vector2(775.79364, 125.79815), Vector2(-3.2922826, 5.2669067)),
      Boid(Vector2(441.56586, 193.37433), Vector2(0.8720325, 1.1982783)),
      Boid(Vector2(70.25492, 92.830185), Vector2(4.759176, -0.27053127)),
      Boid(Vector2(934.1042, 396.42902), Vector2(-4.5564585, 4.131324)),
      Boid(Vector2(307.63562, 210.95468), Vector2(-0.9197886, -5.677174)),
      Boid(Vector2(30.761124, 553.47894), Vector2(-2.8558998, 1.5303923)),
      Boid(Vector2(842.3195, 25.583405), Vector2(-1.4248543, -2.4578722)),
      Boid(Vector2(259.6699, 427.42606), Vector2(-3.8781004, 1.9323616)),
      Boid(Vector2(1.3070107, 183.09727), Vector2(7.086781, 2.6946065)),
      Boid(Vector2(519.7339, 130.86134), Vector2(-1.1993169, -6.310089)),
      Boid(Vector2(885.9177, 13.68072), Vector2(0.20983021, 1.7238382)),
      Boid(Vector2(627.5757, 620.37085), Vector2(1.6819838, -2.461798)),
      Boid(Vector2(832.41925, 70.264915), Vector2(-4.3201985, -4.066921)),
      Boid(Vector2(873.5847, 334.75378), Vector2(2.9220762, -3.1356146)),
      Boid(Vector2(65.18084, 188.8108), Vector2(1.248131, -1.5950269)),
      Boid(Vector2(806.6992, 611.8673), Vector2(-0.47586566, 5.1167707)),
      Boid(Vector2(799.58356, 698.34766), Vector2(-6.178428, 1.7546628)),
      Boid(Vector2(519.31494, 417.9317), Vector2(-1.151279, -1.2084757)),
      Boid(Vector2(681.8049, 519.2394), Vector2(-0.94511676, 3.59132)),
      Boid(Vector2(161.72128, 419.12787), Vector2(3.5077128, 2.3389802)),
      Boid(Vector2(988.7238, 545.08734), Vector2(0.33513477, -3.1563683)),
      Boid(Vector2(279.7233, 518.18866), Vector2(-3.2571878, -0.53243214)),
      Boid(Vector2(453.57834, 207.48364), Vector2(-3.3200457, -4.088408)),
      Boid(Vector2(839.61597, 168.4323), Vector2(-6.3715305, -0.84582496)),
      Boid(Vector2(931.64014, 673.5157), Vector2(-1.9306897, 0.20274377)),
      Boid(Vector2(481.43475, 464.307), Vector2(0.18510936, -7.913158)),
      Boid(Vector2(375.0512, 323.47873), Vector2(2.3692262, 0.7776101)),
      Boid(Vector2(624.4143, 391.01245), Vector2(-5.9428067, 1.4427534)),
      Boid(Vector2(747.1041, 400.68887), Vector2(-1.6449028, 7.7131886)),
      Boid(Vector2(537.8448, 496.13376), Vector2(-0.85485286, -4.192186)),
      Boid(Vector2(573.62665, 116.59546), Vector2(1.9520835, 4.4065332)),
      Boid(Vector2(386.02377, 23.471664), Vector2(-1.0234233, -1.6370882)),
      Boid(Vector2(802.7667, 132.5781), Vector2(-3.017524, -1.6023613)),
      Boid(Vector2(557.7588, 423.37103), Vector2(1.8111383, 0.7241146)),
      Boid(Vector2(524.35596, 24.868643), Vector2(-2.3128827, -3.533858)),
      Boid(Vector2(298.88367, 624.84875), Vector2(-3.0053213, 1.9499414)),
      Boid(Vector2(886.16095, 670.9758), Vector2(0.75396293, 6.181791)),
      Boid(Vector2(780.5093, 222.69064), Vector2(0.13042656, 5.3872814)),
      Boid(Vector2(236.71872, 74.79573), Vector2(0.6832468, -1.7379187)),
      Boid(Vector2(256.23846, 297.43524), Vector2(-1.4287267, -0.77338177)),
      Boid(Vector2(123.079475, 373.27652), Vector2(5.3088155, 1.2935178)),
      Boid(Vector2(888.3304, 347.7817), Vector2(2.6275685, 0.709661)),
      Boid(Vector2(276.6688, 40.064545), Vector2(-3.0312076, 1.0137604)),
      Boid(Vector2(155.80994, 173.45612), Vector2(7.102503, -2.3798523)),
      Boid(Vector2(131.60927, 120.99126), Vector2(0.35302585, 2.6174805)),
      Boid(Vector2(496.77307, 201.94145), Vector2(-7.1780357, 1.2510684)),
      Boid(Vector2(737.22766, 202.57027), Vector2(-6.018245, 0.43434387)),
      Boid(Vector2(982.6496, 304.7225), Vector2(6.741272, -0.19904563)),
      Boid(Vector2(150.10452, 327.8567), Vector2(-7.7128177, -0.5702608)),
      Boid(Vector2(373.94006, 397.8177), Vector2(2.4099867, 1.3653527)),
      Boid(Vector2(520.90686, 334.08017), Vector2(-6.3726516, -4.760052)),
      Boid(Vector2(864.9525, 467.36664), Vector2(-3.8809338, -4.9108877)),
      Boid(Vector2(127.37799, 293.3958), Vector2(-3.3175752, 1.3619006)),
      Boid(Vector2(189.32355, 603.91296), Vector2(-1.3233495, -0.7973769)),
      Boid(Vector2(547.2394, 291.90625), Vector2(-1.3459271, 4.766742)),
      Boid(Vector2(528.6135, 568.76746), Vector2(-4.309171, -6.494619)),
      Boid(Vector2(13.393879, 582.79193), Vector2(4.588395, 3.1074772)),
      Boid(Vector2(843.026, 352.4594), Vector2(0.18353008, 7.5346217)),
      Boid(Vector2(748.70306, 196.94467), Vector2(4.1015334, 0.9068662)),
      Boid(Vector2(826.3079, 418.77728), Vector2(-0.7676945, -0.8902194)),
      Boid(Vector2(738.74133, 188.93369), Vector2(-1.6053218, 1.1020203)),
      Boid(Vector2(676.1453, 558.2743), Vector2(-5.496322, -1.5630165)),
      Boid(Vector2(968.31604, 437.4415), Vector2(-3.2804003, 4.28701)),
      Boid(Vector2(266.87692, 202.35785), Vector2(1.0600775, -1.7617233)),
      Boid(Vector2(621.17206, 7.4911594), Vector2(-3.7777598, -1.0989478)),
      Boid(Vector2(113.10625, 173.20436), Vector2(4.0426197, -0.07336203)),
      Boid(Vector2(122.18553, 516.8442), Vector2(4.915656, 1.0200793)),
      Boid(Vector2(720.79974, 365.7362), Vector2(0.363874, 6.95206)),
      Boid(Vector2(187.48105, 530.8827), Vector2(-0.7442862, 7.592036)),
      Boid(Vector2(536.71814, 75.76993), Vector2(0.5813429, 1.3759242)),
      Boid(Vector2(240.2919, 139.0981), Vector2(0.6405148, -2.383547)),
      Boid(Vector2(727.53296, 503.18878), Vector2(-4.0680084, -4.5340776)),
      Boid(Vector2(649.0139, 41.99099), Vector2(2.0758867, 1.7088382)),
      Boid(Vector2(446.7417, 570.61676), Vector2(-1.267904, 2.081938)),
      Boid(Vector2(248.78848, 499.7475), Vector2(2.0837772, 2.0297413)),
      Boid(Vector2(80.49947, 506.56366), Vector2(-6.1881742, -3.457399)),
      Boid(Vector2(519.07996, 463.48874), Vector2(0.8174308, -1.6400363)),
      Boid(Vector2(202.74513, 47.386055), Vector2(3.4699533, 0.6531648)),
      Boid(Vector2(607.0992, 426.20044), Vector2(3.6679747, 5.164388)),
      Boid(Vector2(615.507, 414.65054), Vector2(-3.3033803, -1.756463)),
      Boid(Vector2(110.55404, 193.63243), Vector2(-4.6253033, 1.0535285)),
      Boid(Vector2(902.0305, 423.61142), Vector2(4.882969, -4.898189)),
      Boid(Vector2(163.09422, 385.5165), Vector2(-0.86981964, -1.9222221)),
      Boid(Vector2(378.37857, 474.50693), Vector2(1.2128085, -7.2351246)),
      Boid(Vector2(901.947, 334.38446), Vector2(7.4486065, -1.0440655)),
      Boid(Vector2(193.4132, 671.25214), Vector2(0.33141914, 3.3228166)),
      Boid(Vector2(839.6261, 253.07768), Vector2(1.6177723, -3.7071638)),
      Boid(Vector2(597.575, 301.85107), Vector2(1.1025367, 0.20199572)),
      Boid(Vector2(10.227144, 492.72192), Vector2(-0.28326488, 2.5507228)),
      Boid(Vector2(77.63362, 501.4726), Vector2(-1.4769937, 3.1896486)),
      Boid(Vector2(15.70648, 10.500574), Vector2(0.3805436, -1.7194685)),
      Boid(Vector2(137.59207, 402.63004), Vector2(0.7935642, 1.9426355)),
      Boid(Vector2(258.67307, 410.99258), Vector2(6.4936686, 0.9472844)),
      Boid(Vector2(995.18414, 57.276512), Vector2(0.022286212, 1.8374891)),
      Boid(Vector2(715.60547, 116.15048), Vector2(5.723291, -3.8295865)),
      Boid(Vector2(358.9123, 392.87274), Vector2(2.2887146, -2.1080527)),
      Boid(Vector2(578.03973, 57.176624), Vector2(0.9044392, -1.7290581)),
      Boid(Vector2(752.33167, 538.6064), Vector2(2.619728, -1.9584509)),
      Boid(Vector2(969.15485, 566.6157), Vector2(0.7540006, -3.2637377)),
      Boid(Vector2(560.29333, 328.93756), Vector2(1.0390625, 3.796842)),
      Boid(Vector2(729.31854, 80.91361), Vector2(-0.38948664, 0.98178136)),
      Boid(Vector2(86.11715, 359.5162), Vector2(-0.7080116, -0.7204893)),
      Boid(Vector2(956.5353, 642.5023), Vector2(2.0403595, 4.3528833)),
      Boid(Vector2(429.17413, 535.2632), Vector2(5.3030553, 2.466011)),
      Boid(Vector2(586.3733, 516.15436), Vector2(-1.9835156, 1.9104548)),
      Boid(Vector2(919.1134, 512.02814), Vector2(1.2340795, 4.073087)),
      Boid(Vector2(800.83075, 106.907906), Vector2(2.3443704, 3.210608)),
      Boid(Vector2(175.9798, 391.73294), Vector2(5.63359, 3.9411364)),
      Boid(Vector2(466.7321, 686.29736), Vector2(0.6715224, -5.307563)),
      Boid(Vector2(604.7226, 86.28147), Vector2(-7.087881, -1.1967217)),
      Boid(Vector2(121.62954, 699.93475), Vector2(-5.3659973, 3.4829705)),
      Boid(Vector2(278.107, 217.76108), Vector2(-1.1578861, 1.2491443)),
      Boid(Vector2(248.81822, 642.6676), Vector2(-3.0329666, 0.38377002)),
      Boid(Vector2(713.7946, 171.58395), Vector2(1.9247946, -0.4719236)),
      Boid(Vector2(712.9543, 647.60065), Vector2(6.916523, 0.28821495)),
      Boid(Vector2(9.370268, 65.21119), Vector2(-4.471133, 3.9243057)),
      Boid(Vector2(263.89987, 442.1997), Vector2(2.2948356, -6.858835)),
      Boid(Vector2(163.34706, 237.97444), Vector2(-5.422306, 5.1129427)),
      Boid(Vector2(958.8015, 278.5076), Vector2(-0.26105893, -6.9206576)),
      Boid(Vector2(180.69167, 607.2557), Vector2(7.258719, 0.30359608)),
      Boid(Vector2(201.03973, 45.182026), Vector2(2.2539768, 0.2827547)),
      Boid(Vector2(738.16907, 672.9066), Vector2(0.313739, 2.656596)),
      Boid(Vector2(849.57263, 616.6273), Vector2(3.8990278, -1.0593497)),
      Boid(Vector2(767.8134, 285.28577), Vector2(-2.337054, 6.0553055)),
      Boid(Vector2(513.32733, 688.92316), Vector2(6.75646, 0.89314175)),
      Boid(Vector2(306.08945, 122.66239), Vector2(0.5550526, 6.6348634)),
      Boid(Vector2(712.6772, 93.56314), Vector2(-6.06982, -3.3630166)),
      Boid(Vector2(351.81714, 550.9813), Vector2(-5.3529563, -2.272404)),
      Boid(Vector2(63.674034, 368.1993), Vector2(-3.3355827, 5.775815)),
      Boid(Vector2(277.69046, 263.10583), Vector2(-7.841511, -1.140725)),
      Boid(Vector2(20.58965, 298.64752), Vector2(-2.6238613, -2.2133386)),
      Boid(Vector2(548.2276, 415.65012), Vector2(1.9631848, -5.3175836)),
      Boid(Vector2(998.4514, 28.341812), Vector2(-2.5801878, 2.5017266)),
      Boid(Vector2(111.248795, 699.385), Vector2(-0.66888833, 2.4879358)),
      Boid(Vector2(889.6409, 360.23474), Vector2(-6.3068314, 4.788074)),
      Boid(Vector2(850.486, 193.94574), Vector2(-1.5097889, -3.4872327)),
      Boid(Vector2(87.25673, 151.82462), Vector2(5.409907, -2.23441)),
      Boid(Vector2(913.6852, 469.19305), Vector2(-1.2836185, 1.7826024)),
      Boid(Vector2(132.82645, 326.54794), Vector2(-5.116043, 6.0921116)),
      Boid(Vector2(563.5712, 204.0734), Vector2(-5.286508, -2.3079736)),
      Boid(Vector2(17.083704, 494.21088), Vector2(3.5775254, 3.7686105)),
      Boid(Vector2(438.30276, 399.19644), Vector2(6.710023, 3.7021494)),
      Boid(Vector2(979.91125, 489.43994), Vector2(2.1033294, 1.3355956)),
      Boid(Vector2(753.55115, 661.59326), Vector2(1.5482779, 0.9136002)),
      Boid(Vector2(379.18198, 500.31793), Vector2(-5.509461, 5.3126388)),
      Boid(Vector2(30.097126, 39.240467), Vector2(-0.8772935, -2.5843942)),
      Boid(Vector2(828.11365, 569.42755), Vector2(3.2568169, -2.3610306)),
      Boid(Vector2(388.8458, 78.93059), Vector2(-4.981704, -4.514338)),
      Boid(Vector2(336.8858, 206.4605), Vector2(2.144022, 4.8604217))
    ),
    chaoticCohesion,
    steps = 0 // Only for display purposes
  )
