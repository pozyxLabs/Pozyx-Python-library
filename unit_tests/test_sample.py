# def test_inputs(remote, interface):
#     print(remote, interface)
#     assert 0

def test_pozyx(pozyx):
    print(pozyx)
    assert 0

def test_2_pozyx(remote, pozyx):
    pozyx.x = remote
    print(pozyx)
    assert 0
