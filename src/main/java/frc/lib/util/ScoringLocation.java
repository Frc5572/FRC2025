package frc.lib.util;

/** Scoring Locations for the 2025 game Reefscape */
public class ScoringLocation {

    /** Reef locations */
    public static enum CoralLocation {
        A, B, C, D, E, F, G, H, I, J, K, L, LeftFeeder, RightFeeder, Processor;

        /** Deserialize from integer */
        public static CoralLocation fromInt(int id) {
            switch (id) {
                case 0:
                    return CoralLocation.A;
                case 1:
                    return CoralLocation.B;
                case 2:
                    return CoralLocation.C;
                case 3:
                    return CoralLocation.D;
                case 4:
                    return CoralLocation.E;
                case 5:
                    return CoralLocation.F;
                case 6:
                    return CoralLocation.G;
                case 7:
                    return CoralLocation.H;
                case 8:
                    return CoralLocation.I;
                case 9:
                    return CoralLocation.J;
                case 10:
                    return CoralLocation.K;
                case 11:
                    return CoralLocation.L;
                case 12:
                    return CoralLocation.LeftFeeder;
                case 13:
                    return CoralLocation.RightFeeder;
                case 14:
                    return CoralLocation.Processor;
                default:
                    return null;
            }
        }
    }

    /** Reef branch heights */
    public static enum CoralHeight {
        L1, L2, L3, L4, Barge;

        /** Deserialize from integer */
        public static CoralHeight fromInt(int id) {
            switch (id) {
                case 0:
                    return CoralHeight.L1;
                case 1:
                    return CoralHeight.L2;
                case 2:
                    return CoralHeight.L3;
                case 3:
                    return CoralHeight.L4;
                case 4:
                    return CoralHeight.L4;
                default:
                    return null;
            }
        }
    }

}
