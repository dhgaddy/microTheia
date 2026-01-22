// See README.md for license details.

ThisBuild / scalaVersion     := "2.13.16"
ThisBuild / version          := "0.1.0"
ThisBuild / organization     := "com.github.mrpologit"

val chiselVersion = "7.0.0"

lazy val root = (project in file("."))
  .settings(
    name := "async-chisel-test",
    libraryDependencies ++= Seq(
      "org.chipsalliance" %% "chisel" % chiselVersion,
      "org.scalatest" %% "scalatest" % "3.2.19" % "test",
    ),
    scalacOptions ++= Seq(
      "-language:reflectiveCalls",
      "-deprecation",
      "-feature",
      "-Xcheckinit",
      "-Ymacro-annotations",
    ),
    addCompilerPlugin("org.chipsalliance" % "chisel-plugin" % chiselVersion cross CrossVersion.full),
    // Compile / unmanagedSources ++= Seq(
    //   baseDirectory.value / "third_party/ASYNC-Chisel/src/main/scala/tool/AsyncLib_ACG.scala",
    //   // baseDirectory.value / "third_party/ASYNC-Chisel/src/main/scala/tool/AnalyzeCircuit.scala"
    // ),
    // Compile / unmanagedResourceDirectories +=
    //   baseDirectory.value / "third_party/ASYNC-Chisel/src/main/resources",
  )
