"""
test_nav_action.py - 测试 nav_action
"""
from nav_action import nav_action, ID_CINIT, ID_ASTART, ID_BSTART, ID_AEND, ID_BEND, ID_TEST

# ---- 测试1: C初始化 ----
print('\n===== 测试1: C初始化 (ID=10) =====')
result = nav_action(ID_CINIT)
print(f'返回: {result}')

# ---- 测试2: Astart ----
# print('\n===== 测试2: Astart (ID=1) =====')
# result = nav_action(ID_ASTART)
# print(f'返回: {result}')

# ---- 测试3: Bstart ----
# print('\n===== 测试3: Bstart (ID=2) =====')
# result = nav_action(ID_BSTART)
# print(f'返回: {result}')

# ---- 测试4: Aend ----
# print('\n===== 测试4: Aend (ID=3) =====')
# result = nav_action(ID_AEND)
# print(f'返回: {result}')

# ---- 测试5: Bend ----
# print('\n===== 测试5: Bend (ID=4) =====')
# result = nav_action(ID_BEND)
# print(f'返回: {result}')

# ---- 测试6: 试管+last=Astart ----
# print('\n===== 测试6: 试管 last=Astart =====')
# result = nav_action(ID_TEST, last_id=ID_ASTART)
# print(f'返回: {result}')

# ---- 测试7: 试管+last=Bstart ----
# print('\n===== 测试7: 试管 last=Bstart =====')
# result = nav_action(ID_TEST, last_id=ID_BSTART)
# print(f'返回: {result}')
