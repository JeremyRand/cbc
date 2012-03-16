/* ---------------------------------------------------------------------------
   commonc++ - A C++ Common Class Library
   Copyright (C) 2005-2009  Mark A Lindner

   This file is part of commonc++.

   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Library General Public
   License as published by the Free Software Foundation; either
   version 2 of the License, or (at your option) any later version.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Library General Public License for more details.

   You should have received a copy of the GNU Library General Public
   License along with this library; if not, write to the Free
   Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
   ---------------------------------------------------------------------------
*/

#ifndef __ccxx_SQLDatabase_hxx
#define __ccxx_SQLDatabase_hxx

#include <commonc++/Common.h++>
#include <commonc++/Blob.h++>
#include <commonc++/IOException.h++>
#include <commonc++/InterruptedException.h++>
#include <commonc++/SharedPtr.h++>
#include <commonc++/String.h++>
#include <commonc++/SQLException.h++>
#include <commonc++/Variant.h++>

namespace ccxx {

class SQLDatabase; // fwd decl
class PreparedStatement; // fwd decl
class SQLitePrivate; // fwd decl

/** A database result set. The next() method should be called to advance to
 * the first and each subsequent row in the ResultSet.
 *
 * @author Mark Lindner
 */

class COMMONCPPDB_API ResultSet
{
  friend class PreparedStatement;

  public:

  /** Destructor. */
  ~ResultSet() throw();

  /** Advance to the next result in the ResultSet.
   *
   * @throw SQLException If a database error occurs.
   * @throw InterruptedException If the operation was interrupted via
   * a call to SQLDatabase::interrupt().
   * @throw TimeoutException If a timeout occurs while waiting for
   * database locks. The call should be retried in this case.
   * @return <b>true</b> if successful, or <b>false</b> if there are no
   * more results.
   */
  bool next() throw(SQLException, InterruptedException, TimeoutException);

  /** Get the number of columns in the ResultSet.
   *
   * @throw SQLException If a database error occurs.
   * @return The column count.
   */
  uint_t getColumnCount() throw(SQLException);

  /** Get the value for the given column in the current row of the
   * ResultSet.
   *
   * @param column The column number (counting from 1).
   * @return The value, as a Variant.
   * @throw SQLException If a database error occurs.
   */
  Variant getColumnValue(uint_t column) throw(SQLException);

  /** Get the unaliased name for the given column in the ResultSet.
   *
   * @param column The column number (counting from 1).
   * @return The column name.
   * @throw SQLException If a database error occurs.
   */
  String getColumnName(uint_t column) throw(SQLException);

  /** Get the aliased name for the given column in the ResultSet.
   *
   * @param column The column number (counting from 1).
   * @return The aliased column name.
   * @throw SQLException If a database error occurs.
   */
  String getAliasedColumnName(uint_t column) throw(SQLException);

  /** Get the name of the table for the given column in the ResultSet.
   *
   * @param column The column number (counting from 1).
   * @return The table name.
   * @throw SQLException If a database error occurs.
   */
  String getTableName(uint_t column) throw(SQLException);

  /** Get the value for the given column in the current row of the
   * ResultSet, as an integer.
   *
   * @param column The column number (counting from 1).
   * @return The value, as an int.
   * @throw SQLException If a database error occurs.
   */
  int getInt(uint_t column) throw(SQLException);

  /** Get the value for the given column in the current row of the
   * ResultSet, as a 64-bit integer.
   *
   * @param column The column number (counting from 1).
   * @return The value, as an int64_t.
   * @throw SQLException If a database error occurs.
   */
  int64_t getInt64(uint_t column) throw(SQLException);

  /** Get the value for the given column in the current row of the
   * ResultSet, as a double-precision floating point value.
   *
   * @param column The column number (counting from 1).
   * @return The value, as a double.
   * @throw SQLException If a database error occurs.
   */
  double getDouble(uint_t column) throw(SQLException);

  /** Get the value for the given column in the current row of the
   * ResultSet, as a String.
   *
   * @param column The column number (counting from 1).
   * @return The value, as a String.
   * @throw SQLException If a database error occurs.
   */
  String getString(uint_t column) throw(SQLException);

  /** Get the value for the given column in the current row of the
   * ResultSet, as a Blob.
   *
   * @param column The column number (counting from 1).
   * @return The value, as a Blob.
   * @throw SQLException If a database error occurs.
   */
  Blob getBlob(uint_t column) throw(SQLException);

  private:

  ResultSet(const SQLitePrivate *private_);
  void prime() throw(SQLException, InterruptedException);
  void _next() throw();

  String _getString(uint_t column) throw();
  Blob _getBlob(uint_t column) throw();

  SQLitePrivate *_private;
  int _result;
  bool _primed;
  bool _hasData;

  CCXX_COPY_DECLS(ResultSet);
};

/** A precompiled SQL statement that can be efficiently executed multiple
 * times.
 *
 * @author Mark Lindner
 */

class COMMONCPPDB_API PreparedStatement
{
  friend class SQLDatabase;
  friend class ResultSet;

  public:

  /** Destructor. */
  ~PreparedStatement() throw();

  /** Reset the statement so that it can be executed again.
   *
   * @throw SQLException If a database error occurs.
   */
  void reset() throw(SQLException);

  /** Get the SQL statement represented by this PreparedStatement.
   *
   * @return The SQL statement, as a String.
   * @throw SQLException If a database error occurs.
   */
  String toSQL() throw(SQLException);

  /** Bind a null value to the designated parameter in the statement.
   *
   * @param index The index of the parameter (where 1 refers to the first
   * parameter).
   * @throw SQLException If a database error occurs.
   */
  void bind(uint_t index) throw(SQLException);

  /** Bind a null value to the designated parameter in the statement.
   *
   * @param name The name of the parameter.
   * @throw SQLException If a database error occurs.
   */
  void bind(const String &name) throw(SQLException);

  /** Bind an integer value to the designated parameter in the statement.
   *
   * @param index The index of the parameter (where 1 refers to the first
   * parameter).
   * @param value The value to bind.
   * @throw SQLException If a database error occurs.
   */
  void bind(uint_t index, int value) throw(SQLException);

  /** Bind an integer value to the designated parameter in the statement.
   *
   * @param name The name of the parameter.
   * @param value The value to bind.
   * @throw SQLException If a database error occurs.
   */
  void bind(const String &name, int value) throw(SQLException);

  /** Bind a 64-bit integer value to the designated parameter in the
   * statement.
   *
   * @param index The index of the parameter (where 1 refers to the first
   * parameter).
   * @param value The value to bind.
   * @throw SQLException If a database error occurs.
   */
  void bind(uint_t index, const int64_t &value) throw(SQLException);

  /** Bind a 64-bit integer value to the designated parameter in the
   * statement.
   *
   * @param name The name of the parameter.
   * @param value The value to bind.
   * @throw SQLException If a database error occurs.
   */
  void bind(const String &name, const int64_t &value) throw(SQLException);

  /** Bind a double-precision floating point value to the designated
   * parameter in the statement.
   *
   * @param index The index of the parameter (where 1 refers to the first
   * parameter).
   * @param value The value to bind.
   * @throw SQLException If a database error occurs.
   */
  void bind(uint_t index, const double &value) throw(SQLException);

  /** Bind a double-precision floating point value to the designated
   * parameter in the statement.
   *
   * @param name The name of the parameter.
   * @param value The value to bind.
   * @throw SQLException If a database error occurs.
   */
  void bind(const String &name, const double &value) throw(SQLException);

  /** Bind a String value to the designated parameter in the
   * statement.
   *
   * @param index The index of the parameter (where 1 refers to the first
   * parameter).
   * @param value The value to bind.
   * @throw SQLException If a database error occurs.
   */
  void bind(uint_t index, const String &value) throw(SQLException);

  /** Bind a String value to the designated parameter in the
   * statement.
   *
   * @param name The name of the parameter.
   * @param value The value to bind.
   * @throw SQLException If a database error occurs.
   */
  void bind(const String &name, const String &value) throw(SQLException);

  /** Bind a Blob value to the designated parameter in the
   * statement.
   *
   * @param index The index of the parameter (where 1 refers to the first
   * parameter).
   * @param value The value to bind.
   * @throw SQLException If a database error occurs.
   */
  void bind(uint_t index, const Blob &value) throw(SQLException);

  /** Bind a Blob value to the designated parameter in the
   * statement.
   *
   * @param name The name of the parameter.
   * @param value The value to bind.
   * @throw SQLException If a database error occurs.
   */
  void bind(const String &name, const Blob &value) throw(SQLException);

  /** Clear the parameter value bindings for the statement.
   *
   * @throw SQLException If a database error occurs.
   */
  void clear() throw(SQLException);

  /** Returns the number of parameters in the statement.
   *
   * @throw SQLException If a database error occurs.
   */
  uint_t getParameterCount() throw(SQLException);

  /** Execute the statement.
   *
   * @return A shared pointer to the ResultSet object which represents
   * the (possibly empty) results.
   * @throw SQLException If a database error occurs.
   * @throw InterruptedException If the operation was interrupted via
   * a call to SQLDatabase::interrupt().
   */
  SharedPtr<ResultSet> execute() throw(SQLException, InterruptedException);

  private:

  PreparedStatement(void *db, void *stmt);
  void _checkValid() throw(SQLException);

  SQLitePrivate *_private;

  CCXX_COPY_DECLS(PreparedStatement);
};

/** A class representing a session with an embedded SQL
 * database. The class can be subclassed to provide custom
 * implementations for the three callback methods.
 *
 * @author Mark Lindner
 */

class COMMONCPPDB_API SQLDatabase
{
  public:

  /** Construct a new SQLDatabase object. */
  SQLDatabase();

  /** Destructor. Closes the database, if it is open. */
  virtual ~SQLDatabase() throw();

  /** Get the version of the underlying SQLite database software. */
  String getVersion() throw();

  /** Open the database.
   *
   * @param path The pathname of the database file to open.
   * @param readOnly A flag indicating whether the database should be
   * opened read-only.
   * @param create A flag indicating whether the database should be
   * created if it does not exist.
   * @throw IOException If the database file could not be opened.
   */
  void open(const String &path, bool readOnly = false, bool create = true)
    throw(IOException);

  /** Close the database. Any pending uncommitted transactions will be
   * rolled back.
   */
  void close() throw();

  /** Execute an SQL statement.
   *
   * @param sql The statement to execute.
   * @return The number of rows that were affected by the statement.
   * @throw SQLException If a database error occurs.
   * @throw InterruptedException If the statement was interrupted via a
   * call to interrupt().
   */
  uint_t executeSQL(const String &sql)
    throw(SQLException, InterruptedException);

  /** Interrupt any currently-executing SQL statement. The corresponding
   * method will throw an InterruptedException.
   *
   * @throw SQLException If a database error occurs.
   */
  void interrupt() throw(SQLException);

  /** Test if the database is currently in auto-commit mode. Auto-commit
   * mode is on by default if a transaction is not currently in progress.
   *
   * @throw SQLException If a database error occurs.
   */
  bool isAutoCommit() throw(SQLException);

  /** Create a prepared statement for the given SQL statement.
   *
   * @param sql The statement.
   * @return The new PreparedStatement. The caller is responsible for
   * deleting this object before closing the database.
   * @throw SQLException If a database error occurs.
   */
  PreparedStatement *prepareStatement(const String &sql) throw(SQLException);

  /** Get the generated key created by the last SQL "INSERT" statement.
   *
   * @return The generated key.
   * @throw SQLException If a database error occurs.
   */
  int64_t getGeneratedKey() throw(SQLException);

  /** Get the number of rows modified by the last SQL statement. */
  uint_t getModifiedRowCount() throw(SQLException);

  /** Begin a transaction. Transactions cannot be nested.
   *
   * @throw SQLException If a database error occurs.
   */
  void begin() throw(SQLException);

  /** Commit a pending transaction.
   *
   * @throw SQLException If a database error occurs.
   * @throw TimeoutException If a timeout occurs while waiting for
   * database locks. The call should be retried in this case.
   */
  void commit() throw(SQLException, TimeoutException);

  /** Roll back a pending transaction.
   *
   * @throw SQLException If a database error occurs.
   * @throw TimeoutException If a timeout occurs while waiting for
   * database locks. The call should be retried in this case.
   */
  void rollback() throw(SQLException, TimeoutException);

  protected:

  /** A callback that's invoked periodically while a SQL statement is
   * being executed. The default implementation does nothing.
   */
  virtual bool onProgress();

  /** A callback that's invoked each time a transaction is committed. The
   * default implementation does nothing.
   */
  virtual void onCommit();

  /** A callback that's invoked each time a transaction is rolled
   * back. The default implementation does nothing.
   */
  virtual void onRollback();

  private:

  void _checkOpened() throw(SQLException);

  static int _progressCallback(void *self);
  static int _commitCallback(void *self);
  static void _rollbackCallback(void *self);
  static int _execCallback(void *self, int cols, char **values,
                           char **colnames);

  class Private; // fwd decl
  Private *_private;
};

/** A scoped database transaction. ScopedTransaction begins a
 * transaction in its constructor, and automatically rolls back the
 * transaction when it goes out of scope, if the transaction had not
 * previously been committed via a call to commit().
 *
 * @author Mark Lindner
 */

class COMMONCPPDB_API ScopedTransaction
{
  public:

  /** Construct a new ScopedTransaction for the given SQLDatabase.
   *
   * @param database The database.
   * @throw SQLException If a database error occurs.
   */
  ScopedTransaction(SQLDatabase &database) throw(SQLException);

  /** Destructor. If the transaction has not been committed via a call
   * commit(), rolls back the transaction.
   */
  ~ScopedTransaction();

  /** Commit the transaction.
   *
   * @throw SQLException If a database error occurs.
   * @throw TimeoutException If a timeout occurs while waiting for
   * database locks. The call should be retried in this case.
   */
  void commit() throw(SQLException, TimeoutException);

  private:

  SQLDatabase &_database;
  bool _committed;
};

}; // namespace ccxx

#endif // __ccxx_SQLDatabase_hxx

/* end of header file */
