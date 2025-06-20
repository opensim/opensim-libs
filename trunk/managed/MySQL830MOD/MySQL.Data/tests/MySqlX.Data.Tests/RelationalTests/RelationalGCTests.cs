// Copyright (c) 2015, 2020, Oracle and/or its affiliates. All rights reserved.
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

using MySqlX.XDevAPI.Relational;
using System;
using NUnit.Framework;

namespace MySqlX.Data.Tests.ResultTests
{
  public class RelationalGCTests : BaseTest
  {
#if NETFRAMEWORK
    [Test]
    public void FetchAllNoReference()
    {
      ExecuteSQL("CREATE TABLE test(name VARCHAR(40), age INT)");
      Table table = testSchema.GetTable("test");

      ExecuteInsertStatement(table.Insert("name", "age").Values("Henry", "22").Values("Patric", 30));
      var result = ExecuteSelectStatement(table.Select());
      var rows = result.FetchAll();
      WeakReference wr = new WeakReference(result);
      result = null;
      GC.Collect();
      Assert.False(wr.IsAlive);
      Assert.AreEqual(2, rows.Count);
      Assert.AreEqual(22, rows[0]["age"]);
      Assert.AreEqual("Patric", rows[1]["name"]);
    }
#endif
  }
}
