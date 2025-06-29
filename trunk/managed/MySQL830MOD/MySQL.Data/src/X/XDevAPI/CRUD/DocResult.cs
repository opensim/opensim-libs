// Copyright (c) 2015, 2023, Oracle and/or its affiliates.
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

using System.Collections.Generic;
using System.Diagnostics;
using MySqlX.XDevAPI.Common;
using MySqlX.Sessions;
using System;
using MySql.Data.MySqlClient;
#if !NETFRAMEWORK
using System.Text.Json;
#endif

namespace MySqlX.XDevAPI.CRUD
{
  /// <summary>
  /// Represents the result of an operation that includes a collection of documents.
  /// <typeparam name="T"/>
  /// </summary>
  public class DocResult<T> : BufferingResult<T>
  {
    System.Text.Encoding _encoding = System.Text.Encoding.UTF8;

    internal DocResult(InternalSession session) : base(session)
    {
      // this is just a single column "doc"
      Debug.Assert(_columns.Count == 1);
    }

    protected override T ReadItem(bool dumping)
    {
      List<byte[]> values = Protocol.ReadRow(this);
      if (values == null) return default;

      Debug.Assert(values.Count == 1);

      if (typeof(T).Name == "DbDoc")
        return (T)Convert.ChangeType(new DbDoc(_encoding.GetString(values[0]).TrimEnd('\0')), typeof(T));
      else
      {
#if !NETFRAMEWORK
        return JsonSerializer.Deserialize<T>(_encoding.GetString(values[0]).TrimEnd('\0'));
#else
        throw new MySqlException("Custom type mapping is only supported from .NET Core 3.1.");
#endif
      }
    }
  }
}
