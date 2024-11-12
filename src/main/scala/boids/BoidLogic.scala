package boids
import cs214.{Vector2, BoidSequence, BoidCons, BoidNil}
import math.{sqrt, acos, Pi, log}

def boidsWithinRadius(thisBoid: Boid, boids: BoidSequence, radius: Float): BoidSequence =
  boids.filter(boid => (boid != thisBoid) && (thisBoid.position.distanceTo(boid.position) <= radius))

/** Force pushing boids away from each other */
/*
def avoidanceForce(thisBoid: Boid, boidsWithinAvoidanceRadius: BoidSequence): cs214.Vector2 =
  if boidsWithinAvoidanceRadius.isEmpty then cs214.Vector2.Zero
  else
    val total_force = boidsWithinAvoidanceRadius.foldLeftVector2(cs214.Vector2.Zero)((accForce, boid) =>
      val diff = thisBoid.position - boid.position
      if diff.norm != 0 then
        val force = diff / diff.squaredNorm
        accForce + force
      else accForce // Ignore boids outside the avoidance radius
    )
    total_force
*/
/*
def mergeBoids(thisBoid: Boid, boids: BoidSequence): BoidSequence =
  val filtered_boids = boids.filter(boid => (boid != thisBoid) && (thisBoid.position.distanceTo(boid.position) <= thisBoid.distanceToMerge))
  if filtered_boids.isEmpty then boids
  else
    val total_position = filtered_boids.foldLeftVector2(cs214.Vector2.Zero)((accForce, boid) => accForce + boid.position)
    val total_velocity = filtered_boids.foldLeftVector2(cs214.Vector2.Zero)((accForce, boid) => accForce + boid.velocity)
    val total_size = filtered_boids.foldLeftFloat(0.0f)((accSize, boid) => accSize + boid.size)
    val total_mass = filtered_boids.foldLeftFloat(0.0f)((accMass, boid) => accMass + boid.mass)
    val total_distanceToMerge = filtered_boids.foldLeftFloat(0.0f)((accForce, boid) => accForce + boid.distanceToMerge)
    val new_position = total_position / filtered_boids.length.toFloat
    val new_velocity = total_velocity / filtered_boids.length.toFloat
    val new_size = total_size / filtered_boids.length.toFloat
    val new_mass = total_mass / filtered_boids.length.toFloat
    val new_distanceToMerge = total_distanceToMerge / filtered_boids.length.toFloat
    val new_boid = Boid(
      position = new_position,
      velocity = new_velocity,
      size = new_size,
      color = thisBoid.color,
      mass = new_mass,
      distanceToMerge = new_distanceToMerge
    )
    val new_boids = boids.filter(boid => filtered_boids.filter(thisBoid => thisBoid != boid).isEmpty)
    mergeBoids(new_boid, new_boids)
*/

def mergeBoids(allBoids: BoidSequence): BoidSequence = {
  var boidsToProcess = allBoids
  var mergedBoids:BoidSequence = BoidNil() //List.empty[Boid]

  while (!boidsToProcess.isEmpty) {
    val thisBoid = boidsToProcess.head
    val nearbyBoids = boidsWithinRadius(thisBoid, boidsToProcess.tail, thisBoid.distanceToMerge)
    val boidsToMerge = BoidCons(thisBoid, nearbyBoids)

    // Remove merged boids from the processing list
    val boidsToRemove = boidsToMerge.filter(boid => (boid != thisBoid))
    boidsToProcess = boidsToProcess.tail.filter(boid => !boidsToMerge.contains(boid))

    // Calculate new properties for the merged boid
    val newBoid = mergeBoidProperties(boidsToMerge.toSeq)
    mergedBoids = BoidCons(newBoid, mergedBoids)
  }

  mergedBoids
}
/*
def mergeBoidProperties(boids: Seq[Boid]): Boid = {
  val count = boids.length.toFloat

  val (totalPosition, totalVelocity, totalSize, totalMass, totalDistanceToMerge) = boids.foldLeft(
    (Vector2.Zero, Vector2.Zero, 0.0f, 0.0f, 0.0f)
  ) { case ((posAcc, velAcc, sizeAcc, massAcc, distAcc), boid) =>
    (
      posAcc + boid.position,
      velAcc + boid.velocity * boid.mass,
      sizeAcc + boid.size,
      massAcc + boid.mass,
      distAcc + boid.distanceToMerge
    )
  }

  Boid(
    position = totalPosition / count,
    velocity = totalVelocity / totalMass,
    size = totalSize, // decrease the growth rate so that they are not too big
    color = boids.head.color,
    mass = totalMass,
    distanceToMerge = totalDistanceToMerge
  )
}
  */

def mergeBoidProperties(boids: Seq[Boid]): Boid = {
  val count = boids.length.toFloat

  val totalMass = boids.foldLeft(0.0f)((acc, boid) => acc + boid.mass)
  val totalPosition = boids.foldLeft(Vector2.Zero)((acc, boid) => acc + boid.position * boid.mass)
  val totalVelocity = boids.foldLeft(Vector2.Zero)((acc, boid) => acc + boid.velocity * boid.mass)
  // val totalDistanceToMerge = boids.foldLeft(0.0f)((acc, boid) => acc + boid.distanceToMerge)
  // val newSize = boids.foldLeft(0.0f)((acc, boid) => acc + boid.size)

  // Parameters for size calculation
  val baseSize = 0.0f         // Base size of a single boid
  val growthFactor = 1.0f      // Adjust this to control growth rate
  val sizeExponent = 0.3f      // Exponent less than 1 for sub-linear growth

  // New size calculation using a sub-linear function
  val newSize = baseSize + growthFactor * math.pow(totalMass, sizeExponent).toFloat
  val newDistanceToMerge = newSize * 2f

  Boid(
    position = totalPosition / totalMass,
    velocity = totalVelocity / totalMass,
    size = newSize,
    color = boids.head.color,
    mass = totalMass,
    distanceToMerge = newSize // newDistanceToMerge / (totalMass * 0.7f)
  )
}


/** Force pushing boids towards each other */
def cohesionForce(thisBoid: Boid, boidsWithinPerceptionRadius: BoidSequence): cs214.Vector2 =
  if boidsWithinPerceptionRadius.isEmpty then cs214.Vector2.Zero
  else
    val total_force = boidsWithinPerceptionRadius.foldLeftVector2(cs214.Vector2.Zero)((accForce, boid) => 
      if boid != thisBoid then accForce + boid.position else accForce)
    total_force/boidsWithinPerceptionRadius.length.toFloat - thisBoid.position

/** Force pushing boids to align with the direction of their neighbors */
def alignmentForce(thisBoid: Boid, boidsWithinPerceptionRadius: BoidSequence): cs214.Vector2 =
  if boidsWithinPerceptionRadius.isEmpty then cs214.Vector2.Zero
  else
    val total_force = boidsWithinPerceptionRadius.foldLeftVector2(cs214.Vector2.Zero)((accForce, boid) =>
      if boid != thisBoid then accForce + boid.velocity else accForce)
    total_force/boidsWithinPerceptionRadius.length.toFloat - thisBoid.velocity

/** Force keeping boids within simulation bounds */
def containmentForce(thisBoid: Boid, limits: BoundingBox): cs214.Vector2 =
  val x = if (thisBoid.position.x < limits.xmin) then 1.0f
    else if (thisBoid.position.x > limits.xmax) then -1.0f
    else 0.0f

  val y = if (thisBoid.position.y < limits.ymin) then 1.0f
    else if (thisBoid.position.y > limits.ymax) then -1.0f
    else 0.0f
  cs214.Vector2(x, y)

def totalForce(thisBoid: Boid, allBoids: BoidSequence, physics: Physics): Vector2 =
  val withinPerceptionRadius = boidsWithinRadius(thisBoid, allBoids, physics.perceptionRadius)
  val cohere = cohesionForce(thisBoid, withinPerceptionRadius)
  val align = alignmentForce(thisBoid, withinPerceptionRadius)
  val withinAvoidanceRadius = boidsWithinRadius(thisBoid, withinPerceptionRadius, physics.avoidanceRadius)
  // val avoid = avoidanceForce(thisBoid, withinAvoidanceRadius)
  val contain = containmentForce(thisBoid, physics.limits)
  val total =
    //avoid * physics.avoidanceWeight * thisBoid.mass +
      cohere * physics.cohesionWeight * thisBoid.mass +
      align * physics.alignmentWeight * thisBoid.mass +
      contain * physics.containmentWeight * thisBoid.mass
  total


/** Returns the given boid, one tick later */
def tickBoid(thisBoid: Boid, allBoids: BoidSequence, physics: Physics): Boid =
  val acceleration = totalForce(thisBoid, allBoids, physics)
  val newVelocity = thisBoid.velocity + acceleration
  val newCorrectedVelocity = 
    if (newVelocity.squaredNorm > physics.maximumSpeed * physics.maximumSpeed) then newVelocity.normalized * physics.maximumSpeed
    else if (newVelocity.squaredNorm < physics.minimumSpeed * physics.minimumSpeed) then newVelocity.normalized * physics.minimumSpeed
    else newVelocity

  val newPosition = thisBoid.position + thisBoid.velocity

  Boid(
    position = newPosition,
    velocity = newCorrectedVelocity,
    size = thisBoid.size,
    color = thisBoid.color,
    mass = thisBoid.mass,
    distanceToMerge = thisBoid.distanceToMerge
  )


/** Returns all the given boids, one tick later */
def tickWorld(allBoids: BoidSequence, physics: Physics): BoidSequence = {
  // allBoids.mapBoid(boid => tickBoid(boid, allBoids, physics))
  // First, merge boids that are close to each other
  val mergedBoids = mergeBoids(allBoids)

  // Then, update each boid for the next tick
  mergedBoids.mapBoid(boid => tickBoid(boid, mergedBoids, physics))
}
