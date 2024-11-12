package boids

import cs214.*

// Import necessary packages
import org.junit.Test
import org.junit.Assert._

class BoidTest extends munit.FunSuite:

  test("world with single boid, no forces"):
    runTestCase("00_singleBoidNoForces")

  test("world with three boids, no forces"):
    runTestCase("01_threeBoidsNoForces")

  // test("avoidance doesn't affect lone boid"):
  //  runTestCase("10_singleBoidAvoidance")
  
  // test("avoidance between two boids face-to-face"):
  //  runTestCase("11_twoBoidsAvoidanceX")

  // test("same as above, with orthogonal velocity component"):
  //  runTestCase("12_twoBoidsAvoidanceXY")

  test("no avoidance between far boids"):
    runTestCase("13_twoBoidsAvoidanceFar")

  // test("avoidance among mixed boids"):
  //  runTestCase("14_mixedAvoidance")

  // test("avoidance among boids at the same position"):
  //  runTestCase("15_avoidanceSamePosition")

  // test("only cohesion between two boids"):
  //   runTestCase("20_twoBoidsRestCohesion")

  test("cohesion makes two boids dance"):
    runTestCase("21_twoBoidsCohesionDance")

  test("no cohesion between two far boids"):
    runTestCase("22_twoBoidsCohesionFar")

  test("cohesion can be chaotic"):
    runTestCase("23_chaoticCohesion")

  test("cohesion outside avoidance range"):
    runTestCase("30_avoidanceCohesion")

  // test("cohesion within avoidance range"):
  //  runTestCase("31_avoidanceCohesionLonger")

  test("the three-body problem"):
    runTestCase("32_threeBodyProblem")

  test("only alignment"):
    runTestCase("40_onlyAlignment")

  test("no alignment between far boids"):
    runTestCase("41_alignmentFar")

  test("containment above top"):
    runTestCase("50_containmentTop")

  test("containment below bottom"):
    runTestCase("51_containmentBottom")

  test("containment beyond left"):
    runTestCase("52_containmentLeft")

  test("containment beyond right"):
    runTestCase("53_containmentRight")

  test("cumulative containment"):
    runTestCase("54_containmentCumulative")

  // test("all forces together on many boids"):
  //  runTestCase("60_allTogether")

  val MergePhysics = Physics(
    limits = Physics.defaultLimits,
    minimumSpeed = 0f,
    maximumSpeed = 4f,
    perceptionRadius = 80f,
    avoidanceRadius = 22f,
    // All weights are zero to isolate merging behavior
    avoidanceWeight = 0f,
    cohesionWeight = 0f,
    alignmentWeight = 0f,
    containmentWeight = 0f
  )

  // Test 1: Testing merging functionality in isolation
  test("just merge boids") {
    val boids = BoidCons(
      Boid(Vector2(0, 0), Vector2(0, 0)),
      BoidCons(
        Boid(Vector2(1, 0), Vector2(1, 1)),
        BoidCons(
          Boid(Vector2(1, 0), Vector2(2, 2)),
          BoidNil()
        )
      )
    )

    val expectedPosition = boids.foldLeftVector2(Vector2.Zero)((acc, boid) => acc + boid.position * boid.mass)
    // val expectedSize = boids.foldLeftFloat(0.0f)((acc, boid) => acc + boid.size)
    val expectedMass = boids.foldLeftFloat(0.0f)((acc, boid) => acc + boid.mass)
    val expectedVelocity = boids.foldLeftVector2(Vector2.Zero)((acc, boid) => acc + boid.velocity * boid.mass)
    val expectedDistanceToMerge = boids.foldLeftFloat(0.0f)((acc, boid) => acc + boid.distanceToMerge)
    // Parameters for size calculation
    val baseSize = 0.0f         // Base size of a single boid
    val growthFactor = 1.0f      // Adjust this to control growth rate
    val sizeExponent = 0.3f      // Exponent less than 1 for sub-linear growth

    // New size calculation using a sub-linear function
    val expectedSize = baseSize + growthFactor * math.pow(expectedMass, sizeExponent).toFloat

    val expectedBoid = Boid(
      position = expectedPosition / expectedMass,
      velocity = expectedVelocity / expectedMass,
      size = expectedSize,
      color = Boids.DEFAULT_COLOR,
      mass = expectedMass,
      distanceToMerge = expectedSize
    )

    val mergedBoids = mergeBoids(boids)

    // Since all boids should merge into one
    val expectedMergedBoids = BoidCons(expectedBoid, BoidNil())

    assertEquals(1, mergedBoids.length)
    assertEquals(expectedMergedBoids.head, mergedBoids.head)
    println("Test 'just merge boids' passed")
  }

  // Test 2: Ensuring boids do not merge when they are not within merging distance
  test("no merge boids") {
    val boids = BoidCons(
      Boid(Vector2(0, 0), Vector2(0, 0)),
      BoidCons(
        Boid(Vector2(100, 100), Vector2(1, 1)),
        BoidCons(
          Boid(Vector2(200, 200), Vector2(2, 2)),
          BoidNil()
        )
      )
    )

    val mergedBoids = mergeBoids(boids).reverse
    // val newBoids = tickWorld(boids, physics = Merge)

    // Expect no merging; the boids list should remain the same
    assertEquals(boids.length, mergedBoids.length)

    // Check that each boid remains unchanged
    def compareBoids(original: BoidSequence, merged: BoidSequence): Unit = {
      if (!original.isEmpty && !merged.isEmpty) {
        assertEquals(original.head.position, merged.head.position)
        assertEquals(original.head.velocity, merged.head.velocity)
        compareBoids(original.tail, merged.tail)
      } else {
        assertTrue(original.isEmpty && merged.isEmpty)
      }
    }

    compareBoids(boids, mergedBoids)
    println("Test 'no merge boids' passed")
  }

  // Test 3: Verifying boids merge and take a step when within merging distance
  test("merge boids with tickWorld") {
    val boids = BoidCons(
      Boid(Vector2(0, 0), Vector2(1, 1)),
      BoidCons(
        Boid(Vector2(1, 0), Vector2(1, 1)),
        BoidCons(
          Boid(Vector2(2, 0), Vector2(1, 1)),
          BoidNil()
        )
      )
    )

    // Take a step in the environment using tickWorld
    val newBoids = tickWorld(boids, MergePhysics)

    // Since all boids are close enough, they should merge into one and then move according to tickBoid

    // Compute expected merged properties before movement
    // val mergedSize = boids.foldLeftFloat(0.0f)((acc, boid) => acc + boid.size)
    val mergedMass = boids.foldLeftFloat(0.0f)((acc, boid) => acc + boid.mass)
    val mergedPosition = boids.foldLeftVector2(Vector2.Zero)((acc, boid) => acc + boid.position * boid.mass) / mergedMass
    val mergedVelocity = boids.foldLeftVector2(Vector2.Zero)((acc, boid) => acc + boid.velocity * boid.mass) / mergedMass
    val mergedDistanceToMerge = boids.foldLeftFloat(0.0f)((acc, boid) => acc + boid.distanceToMerge)

     // Parameters for size calculation
    val baseSize = 0.0f         // Base size of a single boid
    val growthFactor = 1.0f      // Adjust this to control growth rate
    val sizeExponent = 0.3f      // Exponent less than 1 for sub-linear growth

    // New size calculation using a sub-linear function
    val mergedSize = baseSize + growthFactor * math.pow(mergedMass, sizeExponent).toFloat
    // Since all weights are zero, totalForce will be zero, so acceleration is zero
    // Velocity remains the same, and position updates by adding velocity

    // Adjust expected velocity considering speed correction
    val speedSquared = mergedVelocity.squaredNorm
    val maxSpeedSquared = MergePhysics.maximumSpeed * MergePhysics.maximumSpeed

    val expectedVelocity = if (speedSquared > maxSpeedSquared) then
      mergedVelocity.normalized * MergePhysics.maximumSpeed
    else
      mergedVelocity

    val expectedPosition = mergedPosition + mergedVelocity

    val expectedBoid = Boid(
      position = expectedPosition,
      velocity = expectedVelocity,
      size = mergedSize,
      color = boids.head.color,
      mass = mergedMass,
      distanceToMerge = mergedSize
    )

    // The newBoids should contain only the expectedBoid

    assertEquals(1, newBoids.length)
    assertEquals(newBoids.head, expectedBoid)
    println("Test 'merge boids with tickWorld' passed")
  }


