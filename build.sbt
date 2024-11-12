name := "boids"
scalaVersion := "3.5.0"
scalacOptions ++= Seq("-deprecation", "-feature", "-language:fewerBraces", "-Xfatal-warnings")
run / fork := true
Global / cancelable := true

val toolkitVersion = "0.2.1"
val caskVersion = "0.9.4"
libraryDependencies ++= Seq(
  "com.lihaoyi" %% "cask" % caskVersion,
  "org.scala-lang" %% "toolkit" % toolkitVersion,
  "org.scala-lang" %% "toolkit-test" % toolkitVersion % Test,
)

