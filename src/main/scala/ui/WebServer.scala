package ui

// Needed because cask.MainRoutes is not open
import language.adhocExtensions

import cs214.*
import boids.*

import upickle.default.*
import java.awt.Color

object WebServer extends cask.MainRoutes:

  /** Paths where the static content served by the server is stored */
  private val WEB_STATIC_PATH = "src/main/www"
  private def HTML_STATIC_FILE =
    cask.model.StaticFile(WEB_STATIC_PATH + "/boids.html", Seq("Content-Type" -> "text/html"))
  private val TEST_CASES_STATIC_PATH = "src/test/json"

  /** Int color to hex string and converse because only hex strings are
    * exchanged
    */
  private def rgbToHex(rgb: Int): String =
    "#%06X".format(rgb & 0xffffff)
  private def hexToRgb(hex: String): Int =
    Integer.parseInt(hex.stripPrefix("#"), 16)

  /* TODO, cask depends on upickle version 3.0.0, however @upickle.implicits.serializeDefaults(true)
   * is introduced in upickle 4.0.0. If cask changes its dependency, we can replace the RW with this:
   * {{{
   * @upickle.implicits.serializeDefaults(true)
   * case class BoidData(x: Float, y: Float, vx: Float, vy: Float, size: Float = Boid.DEFAULT_SIZE, rgbHex: String = Boid.DEFAULT_COLOR.getRGB())
   * }}}
   */
  case class BoidData(x: Float, y: Float, vx: Float, vy: Float, size: Float, rgbHex: String)
  given BoidDataRW: ReadWriter[BoidData] =
    readwriter[ujson.Value].bimap[BoidData](
      boid =>
        ujson.Obj(
          "x" -> boid.x,
          "y" -> boid.y,
          "vx" -> boid.vx,
          "vy" -> boid.vy,
          "size" -> boid.size,
          "rgb" -> boid.rgbHex
        ),
      json =>
        val obj = json.obj
        BoidData(
          obj("x").num.toFloat,
          obj("y").num.toFloat,
          obj("vx").num.toFloat,
          obj("vy").num.toFloat,
          obj.get("size").map(_.num.toFloat).getOrElse(Boids.DEFAULT_SIZE),
          obj.get("rgb").map(_.str).getOrElse(rgbToHex(Boids.DEFAULT_COLOR.getRGB()))
        )
    )
  case class Config(initialBoids: BoidSequence, physics: Physics) derives ReadWriter
  case class Update(elapsedMs: Double, boids: BoidSequence) derives ReadWriter

  // BoidSequence – JSON reader and writer (upickle)
  given boidSeqRW: ReadWriter[BoidSequence] =
    readwriter[Seq[BoidData]].bimap[BoidSequence](
      boidSeq =>
        def rec(acc: Seq[BoidData], remaining: BoidSequence): Seq[BoidData] = remaining match
          case BoidCons(head, tail) =>
            rec(
              BoidData(
                head.position.x,
                head.position.y,
                head.velocity.x,
                head.velocity.y,
                head.size,
                rgbToHex(head.color.getRGB())
              ) +: acc,
              tail
            )
          case BoidNil() => acc
        rec(Nil, boidSeq)
      ,
      dataSeq =>
        def rec(acc: BoidSequence, remaining: Seq[BoidData]): BoidSequence = remaining match
          case head :: tail => rec(
              BoidCons(
                new Boid(Vector2(head.x, head.y), Vector2(head.vx, head.vy), head.size, Color(hexToRgb(head.rgbHex))),
                acc
              ),
              tail
            )
          case Nil => acc
        rec(BoidNil(), dataSeq)
    )
  given physicsRW: upickle.default.ReadWriter[Physics] = upickle.default.macroRW
  given boundingBoxRW: upickle.default.ReadWriter[BoundingBox] = upickle.default.macroRW

  var world: World = World(Physics.default)
  var boids: BoidSequence = cs214.BoidNil()

  def initializeRandom(physics: Physics, boidsCount: Int) = synchronized {
    world = World(physics)
    boids = World.createRandom(boidsCount, physics)
  }

  def initializeWith(physics: Physics, initialBoids: BoidSequence) = synchronized {
    world = World(physics)
    boids = initialBoids
  }

  def update = synchronized {
    boids = world.tick(boids)
  }

  def withTimer[T](body: => T): (Double, T) =
    val start = System.nanoTime()
    val result = body
    val elapsedMs = (System.nanoTime() - start) / 1e6
    (elapsedMs, result)

  @cask.get("/")
  def getIndexFile() = HTML_STATIC_FILE

  @cask.staticFiles("/src")
  def webStaticFiles() = WEB_STATIC_PATH

  @cask.staticFiles("/testCase")
  def testCaseStaticFileRoute() = TEST_CASES_STATIC_PATH

  @cask.getJson("/get")
  def getStep() =
    val (time, _) = withTimer(update)
    Update(time, boids)

  @cask.getJson("/testCases")
  def getTestCases() = os.list(os.pwd / "src" / "test" / "json").map(_.baseName)

  @cask.postJson("/initializeRandom")
  def postInitializeRandom(boidsCount: Int, physics: Physics) =
    initializeRandom(physics, boidsCount)
    Config(boids, world.physics)

  @cask.postJson("/initializeWith")
  def postInitializeWith(initialBoids: BoidSequence, physics: Physics) =
    initializeWith(physics, initialBoids)
    Config(initialBoids, physics)

  initialize()
