DROP USER IF EXISTS 'test';
DROP USER IF EXISTS 'testNoPass';
DROP USER IF EXISTS 'testSha2';
DROP USER IF EXISTS 'testNative';
DROP USER IF EXISTS 'testAnyhost';
CREATE USER 'test'@'%' identified by 'test';
GRANT ALL PRIVILEGES  ON *.*  TO 'test'@'%';
CREATE USER 'testNoPass'@'%';
CREATE USER 'testSha2'@'%' identified with caching_sha2_password by 'mysql';
GRANT ALL PRIVILEGES  ON *.*  TO 'testSha2'@'%';
CREATE USER 'testNative'@'%' identified with mysql_native_password by 'test';
CREATE USER 'testAnyhost'@'%' identified by 'test';
GRANT ALL PRIVILEGES  ON *.*  TO 'testAnyhost'@'%';
FLUSH PRIVILEGES;