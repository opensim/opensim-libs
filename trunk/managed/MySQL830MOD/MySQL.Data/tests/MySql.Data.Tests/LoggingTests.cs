// Copyright (c) 2013, 2020, Oracle and/or its affiliates. All rights reserved.
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License, version 2.0, as
// published by the Free Software Foundation.
//
// This program is also distributed with certain software (including
// but not limited to OpenSSL) that is licensed under separate terms,
// as designated in a particular file or component or in included license
// documentation.  The authors of MySQL hereby grant you an
// additional permission to link the program and your derivative works
// with the separately licensed software that they have included with
// MySQL.
//
// Without limiting anything contained in the foregoing, this file,
// which is part of MySQL Connector/NET, is also subject to the
// Universal FOSS Exception, version 1.0, a copy of which can be found at
// http://oss.oracle.com/licenses/universal-foss-exception.
//
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
// See the GNU General Public License, version 2.0, for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin St, Fifth Floor, Boston, MA 02110-1301  USA

using System;
using System.Text;
using NUnit.Framework;
using System.Diagnostics;

namespace MySql.Data.MySqlClient.Tests
{
  public class LoggingTests : TestBase
  {
    protected override void Cleanup()
    {
      ExecuteSQL(String.Format("DROP TABLE IF EXISTS `{0}`.Test", Connection.Database));
    }

    [Test]
    public void SimpleLogging()
    {
      ExecuteSQL("CREATE TABLE Test(id INT, name VARCHAR(200))");
      ExecuteSQL("INSERT INTO Test VALUES (1, 'Test1')");
      ExecuteSQL("INSERT INTO Test VALUES (2, 'Test2')");
      ExecuteSQL("INSERT INTO Test VALUES (3, 'Test3')");
      ExecuteSQL("INSERT INTO Test VALUES (4, 'Test4')");

      MySqlTrace.Listeners.Clear();
      MySqlTrace.Switch.Level = SourceLevels.All;


      GenericListener listener = new GenericListener();

      MySqlTrace.Listeners.Add(listener);

      using (var logConn = new MySqlConnection(Connection.ConnectionString + ";logging=true"))
      {
        logConn.Open();
        MySqlCommand cmd = new MySqlCommand("SELECT * FROM Test", logConn);
        using (MySqlDataReader reader = cmd.ExecuteReader())
        {
        }
      }
      //Assert.AreEqual(4, listener.Strings.Count);
      Assert.AreEqual(27, listener.Strings.Count);
      StringAssert.Contains("Query Opened: SELECT * FROM Test", listener.Strings[listener.Strings.Count - 5]);
      StringAssert.Contains("Resultset Opened: field(s) = 2, affected rows = -1, inserted id = -1", listener.Strings[listener.Strings.Count - 4]);
      StringAssert.Contains("Resultset Closed. Total rows=4, skipped rows=4, size (bytes)=32", listener.Strings[listener.Strings.Count - 3]);
      StringAssert.Contains("Query Closed", listener.Strings[listener.Strings.Count - 2]);
    }

    [Test]
    public void Warnings()
    {
      ExecuteSQL("CREATE TABLE Test(id INT, name VARCHAR(5))");
      MySqlTrace.Listeners.Clear();
      MySqlTrace.Switch.Level = SourceLevels.All;
      GenericListener listener = new GenericListener();

      MySqlTrace.Listeners.Add(listener);

      using (var logConn = new MySqlConnection(Connection.ConnectionString + ";logging=true"))
      {
        logConn.Open();
        MySqlCommand cmd = new MySqlCommand("INSERT IGNORE INTO Test VALUES (1, 'abcdef')", logConn);
        cmd.ExecuteNonQuery();
      }

      Assert.AreEqual(32, listener.Strings.Count);
      StringAssert.Contains("Query Opened: INSERT IGNORE INTO Test VALUES (1, 'abcdef')", listener.Strings[listener.Strings.Count - 10]);
      StringAssert.Contains("Resultset Opened: field(s) = 0, affected rows = 1, inserted id = 0", listener.Strings[listener.Strings.Count - 9]);
      StringAssert.Contains("Resultset Closed. Total rows=0, skipped rows=0, size (bytes)=0", listener.Strings[listener.Strings.Count - 8]);
      StringAssert.Contains("Query Opened: SHOW WARNINGS", listener.Strings[listener.Strings.Count - 7]);
      StringAssert.Contains("Resultset Opened: field(s) = 3, affected rows = -1, inserted id = -1", listener.Strings[listener.Strings.Count - 6]);
      StringAssert.Contains("Resultset Closed. Total rows=1, skipped rows=0, size (bytes)=55", listener.Strings[listener.Strings.Count - 5]);
      StringAssert.Contains("Query Closed", listener.Strings[listener.Strings.Count - 4]);
      StringAssert.Contains("MySql Warning: Level=Warning, Code=1265, Message=Data truncated for column 'name' at row 1", listener.Strings[listener.Strings.Count - 3]);
      StringAssert.Contains("Query Closed", listener.Strings[listener.Strings.Count - 2]);
    }

    [Test]
    public void ProviderNormalizingQuery()
    {
      MySqlTrace.Listeners.Clear();
      MySqlTrace.Switch.Level = SourceLevels.All;
      GenericListener listener = new GenericListener();
      MySqlTrace.Listeners.Add(listener);

      StringBuilder sql = new StringBuilder("SELECT '");
      for (int i = 0; i < 400; i++)
        sql.Append("a");
      sql.Append("'");

      using (var logConn = new MySqlConnection(Connection.ConnectionString + ";logging=true"))
      {
        logConn.Open();
        MySqlCommand cmd = new MySqlCommand(sql.ToString(), logConn);
        cmd.ExecuteNonQuery();
      }      

      Assert.AreEqual(28, listener.Strings.Count);
      StringAssert.EndsWith("SELECT ?", listener.Strings[listener.Strings.Count - 5]);
    }

    /// <summary>
    /// Bug #57641	Substring out of range exception in ConsumeQuotedToken
    /// </summary>
    [Test]
    public void QuotedTokenAt300()
    {
      MySqlTrace.Listeners.Clear();
      MySqlTrace.Switch.Level = SourceLevels.All;
      GenericListener listener = new GenericListener();
      MySqlTrace.Listeners.Add(listener);

      string sql = @"SELECT 1 AS `AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA1`,  2 AS `AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA2`,
                3 AS `AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA3`,  4 AS `AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA4`,
                5 AS `AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA5`,  6 AS `AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA6`;";

      using (var logConn = new MySqlConnection(Connection.ConnectionString + ";logging=true"))
      {
        logConn.Open();
        MySqlCommand cmd = new MySqlCommand(sql, logConn);
        using (MySqlDataReader reader = cmd.ExecuteReader())
        {
        }
      }
    }
  }
}