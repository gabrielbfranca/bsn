import xml.etree.ElementTree as ET
class GtestReport:
    def __init__(self, path):
        self.xml_data =self._getXml(path)
        self.root = ET.fromstring(self.xml_data)
    def _getXml(self, path):
        with open(path, "r") as f:
            xml_data = f.read()
        return xml_data
    def isSucessful(self):
        testsuites = self.root.attrib
        total_failures = int((testsuites.get("failures")).strip())
        return total_failures == 0
    def _isTestSuiteSucessful(self, testSuite):
        suite_failures = int((testSuite.get("failures")).strip())
        return suite_failures == 0
    
    def find_testsuite_by_name(self, root, name):
        for testsuite in root.findall("testsuite"):
            if testsuite.attrib.get("name") == name:
                return testsuite
        return None
    def find_testcase_by_name(self, testsuite, name):
        for testcase in testsuite.findall("testcase"):
            if testcase.attrib.get("name") == name:
                return testcase
        return None
    def has_failure_tag(self, testcase):
        failures = testcase.findall("failure")
        return [failure.get('message') for failure in failures]
    def assertTestSuite(self, testSuite):
        if self.isSucessful():
            return (True, '')
        # return error if testSuite is none
        testSuiteTag = self.find_testsuite_by_name(self.root, testSuite)
        if testSuiteTag is None:
            return (False, f"{testSuite} not found")
        if self._isTestSuiteSucessful(testSuiteTag):
            return (True, '')        
        for testcase in testSuiteTag.findall("testcase"):
            assertion, err = self.assertEachTestCase(testSuiteTag, testcase)
            if not assertion:
                return (False, err)
    def assertEachTestCase(self, testSuiteTag ,testCase):
        testCaseTag = self.find_testcase_by_name(testSuiteTag, testCase)
        if testCaseTag is None:
            return (False, f"{testCase} not found")
        failure = self.has_failure_tag(testCaseTag)
        if failure:
            return (False, f"error in {testCaseTag.attrib}: {'FAILURE: '.join(failure)}")
        else:
            return (True, '')
        
    def assertTestCase(self, testSuite, testCase):
        if self.isSucessful():
            return (True, '')
        # return error if testSuite is none
        testSuiteTag = self.find_testsuite_by_name(self.root, testSuite)
        if testSuiteTag is None:
            return (False, f"{testSuite} not found")
        if self._isTestSuiteSucessful(testSuiteTag):
            return (True, '')
        return self.assertEachTestCase(testSuiteTag ,testCase)
# initialize example            
# report = GtestReport(r"especialisra\xmlTest.xml")      

