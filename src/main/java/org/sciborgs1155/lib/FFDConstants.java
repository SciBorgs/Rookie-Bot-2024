package org.sciborgs1155.lib;

public class FFDConstants {
  public final double kS;
  public final double kV;
  public final double kA;

  public FFDConstants(double kS, double kV, double kA) {
    this.kS = kS;
    this.kV = kV;
    this.kA = kA;
  }

  public FFDConstants(double kV, double kA) {
    this(0.0, kV, kA);
  }

  public FFDConstants(double kV) {
    this(0.0, kV, 0.0);
  }
}
