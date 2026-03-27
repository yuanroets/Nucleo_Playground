# 08 Power Safety NUCLEO L476RG

## Golden rule

Never guess power routing or voltage rails.

## Safe default during development

Power board from ST-LINK USB only.

## Before external supply or battery

1. Read NUCLEO-L476RG board manual power section.
2. Confirm the exact input pin and allowed voltage.
3. Confirm jumper positions for selected source.
4. Ensure common ground between supply and board.
5. Avoid conflicting simultaneous power paths.

## LiPo caution

1. Raw LiPo voltage is not constant.
2. Use proper regulation where required.
3. Do not connect raw battery directly unless board path explicitly supports it.

## Pre-power checklist

1. What voltage is being applied
2. Which pin receives it
3. Which jumper selects that route
4. Is GND common and correct
5. Is there any second power source active
