import mock
import nose
from nose.tools import assert_equals
from rws_rviz import DepthCloudRunner

@mock.patch('subprocess.Popen')
def testOneUserRunsAndStops(Popen):
    dc = DepthCloudRunner()
    name = dc.run('/depth1', '/rgb1', 'client1')
    assert_equals('/depth1__rgb1', name)

    cc = {'/depth1': ['client1']}
    assert_equals(cc, dc._converter_clients)
    assert_equals(1, len(dc._converter_processes))
    ec = {('/depth1', '/rgb1'): ['client1']}
    assert_equals(ec, dc._encoder_clients)
    assert_equals(True, ('/depth1', '/rgb1') in dc._encoder_processes)
    assert_equals(1, len(dc._encoder_processes))
    assert_equals(2, Popen.call_count)

    dc.stop('/depth1', '/rgb1', 'client1')
    
    cc = {}
    assert_equals(cc, dc._converter_clients)
    assert_equals(0, len(dc._converter_processes))
    ec = {}
    assert_equals(ec, dc._encoder_clients)
    assert_equals(0, len(dc._encoder_processes))


@mock.patch('subprocess.Popen')
def testOneUserLeavesOneUserStays(Popen):
    dc = DepthCloudRunner()
    name1 = dc.run('/depth1', '/rgb1', 'client1')
    name2 = dc.run('/depth1', '/rgb1', 'client2')
    assert_equals('/depth1__rgb1', name1)
    assert_equals('/depth1__rgb1', name2)

    cc = {'/depth1': ['client1', 'client2']}
    assert_equals(cc, dc._converter_clients)
    assert_equals(True, '/depth1' in dc._converter_processes)
    assert_equals(1, len(dc._converter_processes))
    ec = {('/depth1', '/rgb1'): ['client1', 'client2']}
    assert_equals(ec, dc._encoder_clients)
    assert_equals(True, ('/depth1', '/rgb1') in dc._encoder_processes)
    assert_equals(1, len(dc._encoder_processes))

    dc.stop('/depth1', 'rgb1', 'client1')

    cc = {'/depth1': ['client2']}
    assert_equals(cc, dc._converter_clients)
    assert_equals(True, '/depth1' in dc._converter_processes)
    assert_equals(1, len(dc._converter_processes))
    ec = {('/depth1', '/rgb1'): ['client2']}
    assert_equals(ec, dc._encoder_clients)
    assert_equals(True, ('/depth1', '/rgb1') in dc._encoder_processes)
    assert_equals(1, len(dc._encoder_processes))
