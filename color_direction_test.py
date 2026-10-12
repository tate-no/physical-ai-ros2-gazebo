import sys

positions = {
    'RED': 342,
    'BLUE': 256,
    'YELLOW': 431
}

target = sys.argv[1].upper() if len(sys.argv) > 1 else 'RED'

if target not in positions:
    print('指定できる色: RED, BLUE, YELLOW')
    sys.exit(1)

center = 320
error = positions[target] - center

print(f'目標色: {target}')
print(f'検出位置: X={positions[target]}')
print(f'中央からのずれ: {error:+d} px')

if abs(error) <= 10:
    print('判定: 中央に到達。回転不要')
elif error > 0:
    print('判定: 正の角速度で回転')
else:
    print('判定: 負の角速度で回転')

print('※今回は計算のみ。ロボットは動きません。')
