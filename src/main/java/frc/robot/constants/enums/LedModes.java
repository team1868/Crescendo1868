package frc.robot.constants.enums;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import frc.robot.subsystems.Leds;
import java.util.function.BiFunction;

public enum LedModes {
  ORANGE_COLOR_FLOW(
      (Integer length, Integer offset
      ) -> new ColorFlowAnimation(250, 182, 10, Leds.WHITE, 0.7, length, Direction.Forward, offset)
  ),
  YELLOW_COLOR_FLOW(
      (Integer length, Integer offset
      ) -> new ColorFlowAnimation(250, 238, 10, Leds.WHITE, 0.7, length, Direction.Forward, offset)
  ),
  GREEN_COLOR_FLOW(
      (Integer length, Integer offset
      ) -> new ColorFlowAnimation(10, 250, 50, Leds.WHITE, 0.7, length, Direction.Forward, offset)
  ),
  BLUE_COLOR_FLOW(
      (Integer length, Integer offset
      ) -> new ColorFlowAnimation(10, 90, 250, Leds.WHITE, 0.7, length, Direction.Forward, offset)
  ),
  PURPLE_COLOR_FLOW(
      (Integer length, Integer offset
      ) -> new ColorFlowAnimation(132, 21, 230, Leds.WHITE, 0.7, length, Direction.Forward, offset)
  ),
  RED_SOLID(
      (Integer length,
       Integer offset) -> new SingleFadeAnimation(255, 0, 0, Leds.WHITE, 0, length, offset)
  ),
  ORANGE_SOLID(
      (Integer length,
       Integer offset) -> new SingleFadeAnimation(255, 165, 0, Leds.WHITE, 0, length, offset)
  ),
  GREEN_SOLID(
      (Integer length,
       Integer offset) -> new SingleFadeAnimation(0, 255, 0, Leds.WHITE, 0, length, offset)
  ),
  VARIABLE_SPEED_RED_SOLID(
      (Integer length,
       Integer offset) -> new SingleFadeAnimation(255, 0, 0, Leds.WHITE, 0, length, offset)
  ),
  AUTON_SOLID(
      (Integer length,
       Integer offset) -> new SingleFadeAnimation(255, 202, 250, Leds.WHITE, 0, length, offset)
  ),
  AUTON_BLINKING_1( // far away
      (Integer length,
       Integer offset) -> new StrobeAnimation(225, 202, 250, Leds.WHITE, 0.33, length, offset)
  ),
  AUTON_BLINKING_2( // closer
      (Integer length,
       Integer offset) -> new StrobeAnimation(225, 202, 250, Leds.WHITE, 0.66, length, offset)
  ),
  AUTON_BLINKING_3( // really close
      (Integer length,
       Integer offset) -> new StrobeAnimation(225, 202, 250, Leds.WHITE, 0.7, length, offset)
  ),
  PURPLE_SOLID(
      (Integer length,
       Integer offset) -> new SingleFadeAnimation(111, 3, 252, Leds.WHITE, 0, length, offset)
  ),
  YELLOW_SOLID(
      (Integer length,
       // temporarily becoming purple
       Integer offset) -> new SingleFadeAnimation(128, 0, 128, Leds.WHITE, 0, length, offset)
  ),
  SPOOKIES_BLUE_SOLID(
      (Integer length,
       Integer offset) -> new SingleFadeAnimation(245, 213, 56, Leds.WHITE, 0, length, offset)
  ),
  SPOOKIES_BLUE_LARSON(
      (Integer length, Integer offset
      ) -> new LarsonAnimation(245, 213, 56, Leds.WHITE, 0.7, length, BounceMode.Front, 2, offset)
  ),
  SPOOKIES_BLUE_BLINKING(
      (Integer length,
       Integer offset) -> new StrobeAnimation(225, 202, 250, Leds.WHITE, 0.1, length, offset)
  ),
  YELLOW_BLINKING(
      (Integer length,
       Integer offset) -> new StrobeAnimation(255, 255, 0, Leds.WHITE, 0.1, length, offset)
  ),
  GREEN_BLINKING(
      (Integer length,
       Integer offset) -> new StrobeAnimation(0, 255, 0, Leds.WHITE, 0.1, length, offset)
  ),
  ORANGE_BLINKING(
      (Integer length,
       Integer offset) -> new StrobeAnimation(247, 171, 64, Leds.WHITE, 0.1, length, offset)
  ),
  FOREST_GREEN_BLINKING(
      (Integer length,
       Integer offset) -> new StrobeAnimation(17, 69, 13, Leds.WHITE, 0.1, length, offset)
  ),
  WHITE_BLINKING(
      (Integer length,
       Integer offset) -> new StrobeAnimation(255, 255, 255, Leds.WHITE, 0.1, length, offset)
  ),
  PINK_BLINKING(
      (Integer length,
       Integer offset) -> new StrobeAnimation(250, 173, 167, Leds.WHITE, 0.1, length, offset)
  ),
  VARIABLE_SPEED_PINK_BLINKING(
      (Integer length,
       Integer offset) -> new StrobeAnimation(250, 173, 167, Leds.WHITE, 0.1, length, offset)
  ),
  CYAN_BLINKING(
      (Integer length,
       Integer offset) -> new StrobeAnimation(173, 237, 220, Leds.WHITE, 0.1, length, offset)
  ),
  BROWN_BLINKING(
      (Integer length,
       Integer offset) -> new StrobeAnimation(115, 59, 17, Leds.WHITE, 0.1, length, offset)
  ),
  VARIABLE_SPEED_BROWN_BLINKING(
      (Integer length,
       Integer offset) -> new StrobeAnimation(115, 59, 17, Leds.WHITE, 0.1, length, offset)
  ),
  LIME_GREEN_BLINKING(
      (Integer length,
       Integer offset) -> new StrobeAnimation(15, 237, 7, Leds.WHITE, 0.1, length, offset)
  ),
  VARIABLE_SPEED_LIME_GREEN_BLINKING(
      (Integer length,
       Integer offset) -> new StrobeAnimation(15, 237, 7, Leds.WHITE, 0.1, length, offset)
  ),
  WHITE_SOLID(
      (Integer length,
       Integer offset) -> new SingleFadeAnimation(0, 0, 0, Leds.WHITE, 0, length, offset)
  ),
  COLOR_FLOW(
      (Integer length, Integer offset
      ) -> new ColorFlowAnimation(255, 0, 0, Leds.WHITE, 0.7, length, Direction.Forward, offset)
  ),
  RGB_FADE((Integer length, Integer offset) -> new RgbFadeAnimation(1, 0.7, length, offset)),
  FIRE(
      (Integer length,
       Integer offset) -> new FireAnimation(Leds.BRIGHTNESS, 0.1, length, 0.7, 0.7, false, offset)
  ),
  LARSON_ORANGE(
      (Integer length, Integer offset
      ) -> new LarsonAnimation(252, 132, 3, Leds.WHITE, 0.7, length, BounceMode.Front, 2, offset)
  ),
  VARIABLE_SPEED_LARSON_ORANGE(
      (Integer length, Integer offset
      ) -> new LarsonAnimation(252, 132, 3, Leds.WHITE, 0.7, length, BounceMode.Front, 2, offset)
  ),
  LARSON_GREEN(
      (Integer length, Integer offset
      ) -> new LarsonAnimation(9, 99, 20, Leds.WHITE, 0.7, length, BounceMode.Front, 2, offset)
  ),
  VARIABLE_SPEED_LARSON_GREEN(
      (Integer length, Integer offset
      ) -> new LarsonAnimation(9, 99, 20, Leds.WHITE, 0.7, length, BounceMode.Front, 2, offset)
  ),
  LARSON_PINK(
      (Integer length, Integer offset
      ) -> new LarsonAnimation(235, 52, 164, Leds.WHITE, 0.7, length, BounceMode.Front, 2, offset)
  ),
  RAINBOW(
      (Integer length,
       Integer offset) -> new RainbowAnimation(Leds.BRIGHTNESS, 0.6, length, false, offset)
  );

  public final BiFunction<Integer, Integer, Animation> _animationSupplier;
  public final Animation _fullAnimation;

  LedModes(BiFunction<Integer, Integer, Animation> animationSupplier) {
    _animationSupplier = animationSupplier;
    _fullAnimation = animationSupplier.apply(-1, 0);
  }

  // public LedModes getDecr() {
  //   int prev = (this == COLOR_FLOW ? LedModes.values().length : ordinal()) - 1;
  //   return LedModes.values()[prev];
  // }
}
