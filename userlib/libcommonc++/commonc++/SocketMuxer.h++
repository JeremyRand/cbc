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

#ifndef __ccxx_SocketMuxer_hxx
#define __ccxx_SocketMuxer_hxx

#include <commonc++/Common.h++>
#include <commonc++/CircularBuffer.h++>
#include <commonc++/CriticalSection.h++>
#include <commonc++/Iterator.h++>
#include <commonc++/StaticObjectPool.h++>
#include <commonc++/ServerSocket.h++>
#include <commonc++/StreamSocket.h++>
#include <commonc++/Thread.h++>
#include <commonc++/Mutex.h++>

#ifdef CCXX_OS_POSIX
#include <sys/select.h>
#endif

namespace ccxx {

/** An abstract object representing a network connection. It holds a
 * reference to a connected <b>StreamSocket</b>, and is intended to be
 * subclassed to include application-specific data and/or logic associated
 * with the connection.
 *
 * @author Mark Lindner
 */
class COMMONCPP_API Connection
{
  friend class SocketMuxer;

  public:

  /** The default I/O buffer size. */
  static const size_t DEFAULT_BUFFER_SIZE;

  /** Destructor. */
  virtual ~Connection() throw();

  /** Write data on the connection. The data is enqueued for
   * transmission on the connection.
   *
   * @param buffer The buffer containing the data to be sent.
   * @return <b>true</b> if the data was successfully enqueued, <b>false</b>
   * if there was not enough room in the output buffer to enqueue the
   * data.
   */
  bool writeData(ByteBuffer& buffer);

  /** Write data on the connection. The data is enqueued for
   * transmission on the connection.
   *
   * @param buf The buffer containing the data to be sent.
   * @param count The number of elements to write.
   * @return <b>true</b> if the data was successfully enqueued, <b>false</b>
   * if there was not enough room in the output buffer to enqueue the
   * data.
   */
  bool writeData(const byte_t *buf, size_t count);

  /** Write a "line" of text followed by a CR+LF terminator on the
   * connection.
   *
   * @param text The text to write.
   * @return <b>true</b> if the data was successfully enqueued, <b>false</b>
   * if there was not enough room in the output buffer to enqueue the
   * data.
   */
  bool writeLine(const String &text);

  /** Read data on the connection. Reads data that has already been received
   * on the connection.
   *
   * @param buffer The buffer to which the data should be written.
   * @param fully If <b>true</b>, fail if there is not enough data available
   * to fill the buffer.
   * @return The number of bytes read.
   */
  size_t readData(ByteBuffer& buffer, bool fully = true);

  /** Read data on the connection. Reads data that has already been received
   * on the connection.
   *
   * @param buf The buffer to which the data should be written.
   * @param count The number of bytes to read.
   * @param fully If <b>true</b>, fail if there is not enough data available
   * to fill the buffer.
   * @return The number of bytes read.
   */
  size_t readData(byte_t *buf, size_t count, bool fully = true);

  /** Read a "line" of text followed by a CR+LF terminator on the
   * connection. The terminator is not discarded.
   *
   * @param text The String to read the text into.
   * @param maxLen The maximum number of characters to read (including the
   * CR+LF terminator).
   * @return The number of characters read.
   */
  size_t readLine(String &text, size_t maxLen);

  /** Get the socket for this connection. */
  inline StreamSocket* getSocket() throw()
  { return(_socket); }

  /** Set the read low-water mark for the connection. This is the
   * minimum amount of data that must be available in the input
   * buffer before the <b>SocketMuxer::dataReceived()</b> callback
   * is invoked for the connection. The default value is 1.
   *
   * @param count The low-water mark, in bytes.
   */
  void setReadLowWaterMark(size_t count) throw();

  /** Set the read high-water mark for the connection. If the amount
   * of data in the input buffer reaches or exceeds the high-water
   * mark, the SocketMuxer will stop attempting to receive more data
   * on this connection until the amount of data in the input buffer
   * falls below the high-water mark.
   *
   * @param count The low-water mark, in bytes.
   */
  void setReadHighWaterMark(size_t count) throw();

  /** Get the current value of the read low-water mark.
   *
   * @return The low-water mark, in bytes.
   */
  inline size_t getReadLowWaterMark() const throw()
  { return(_readLoMark); }

  /** Test if the amount of data available to be read on the connection
   * is less than the read low-water mark.
   */
  bool isReadLow() const throw();

  /** Test if the amount of data available to be read on the connection
   * is greater than or equal to the read high-water mark.
   */
  bool isReadHigh() const throw();

  /** Set the write low-water mark for the connection. If the amount
   * of data queued in the output buffer drops below the high-water
   * mark, the SocketMuxer will stop attempting to transmit the
   * queued data over the connection until the amount of data in the
   * output buffer rises above the low-water mark. The default value
   * is 1.
   *
   * @param count The low-water mark, in bytes.
   */
  void setWriteLowWaterMark(size_t count) throw();

  /** Set the write high-water mark for the connection. The SocketMuxer
   * does not use this value directly, but subclasses of Connection may
   * choose to use it when implementing an output throttling strategy.
   *
   * @param count The high-water mark, in bytes.
   */
  void setWriteHighWaterMark(size_t count) throw();

  /** Get the current value of the write low-water mark.
   *
   * @return The low-water mark, in bytes.
   */
  inline size_t getWriteLowWaterMark() const throw()
  { return(_writeLoMark); }

  /** Get the current value of the write high-water mark.
   *
   * @return The high-water mark, in bytes.
   */
  inline size_t getWriteHighWaterMark() const throw()
  { return(_writeHiMark); }

  /** Test if the amount of data queued to be written on the
   * connection is less than the write low-water mark.
   */
  bool isWriteLow() const throw();

  /** Test if the amount of data queued to be written on the
   * connection is greater than or equal to the write high-water
   * mark.
   */
  bool isWriteHigh() const throw();

  /** Close the connection.
   *
   * @param immediate If <b>true</b>, close the connection immediately,
   * even if data is still in the write buffer; otherwise, close the
   * connection after all pending data has been written.
   */
  void close(bool immediate = false) throw();

  /** Get any pending out-of-band data byte, and clear the OOB flag.
   *
   * @return The most recent byte of OOB data received.
   */
  inline byte_t getOOBData() throw()
  { _oobFlag = false; return(_oobData); }

  /** Test the out-of-band flag.
   *
   * @return <b>true</b> if a byte of OOB data is pending, <b>false</b>
   * otherwise.
   */
  inline bool getOOBFlag() const throw()
  { return(_oobFlag); }

  /** Get the time at which data was last received on this connection.
   */
  inline time_ms_t getTimestamp() const throw()
  { return(_lastRecv); }

  /** Test if a close is pending on the connection.
   */
  bool isClosePending() const throw();

  protected:

  /** Construct a new <b>Connection</b>.
   *
   * @param bufferSize The size for the I/O buffers.
   */
  Connection(size_t bufferSize = DEFAULT_BUFFER_SIZE);

  private:

  void read() throw(IOException);
  void readOOB() throw(IOException);
  void write() throw(IOException);

  void attach(StreamSocket *socket) throw();

  inline void setOOBFlag(bool flag) throw()
  {  _oobFlag = flag; }

  inline void setTimestamp(time_ms_t stamp) throw()
  { _lastRecv = stamp; }

  StreamSocket *_socket;

  protected:

  CircularByteBuffer readBuffer;
  CircularByteBuffer writeBuffer;

  private:

  size_t _readLoMark;
  size_t _readHiMark;
  size_t _writeLoMark;
  size_t _writeHiMark;
  bool _oobFlag;
  bool _closePending;
  byte_t _oobData;
  time_ms_t _lastRecv;
  mutable CriticalSection _readLock;
  mutable CriticalSection _writeLock;

  CCXX_COPY_DECLS(Connection);
};

/** A socket I/O multiplexer. This class is built around the Reactor
 * pattern; the various connection event handlers are called when
 * the corresponding I/O events occur on the sockets being managed
 * by the muxer. A muxer must run in its own thread.
 *
 * @author Mark Lindner
 */

class COMMONCPP_API SocketMuxer : public Thread
{
  public:

  /** Construct a new SocketMuxer.
   *
   * @param maxConnections The maximum number of connections that
   * the muxer should manage.
   * @param defaultIdleLimit The default idle limit for connections,
   * in milliseconds. Connections that exceed their idle limit will
   * be closed automatically. A value of 0 indicates no idle limit.
   * @param sleepInterval The maximum amount of time, in milliseconds,
   * to wait in the I/O loop for one or more connections to become
   * ready for reading or writing.
   */

  SocketMuxer(uint_t maxConnections = 64, uint_t defaultIdleLimit = 0,
              uint_t sleepInterval = 100);

  /** Destructor. Closes and destroys all active connections. */

  virtual ~SocketMuxer() throw();

  void run();
  void cleanup();

  /** Get the count of currently active connections. */

  size_t getConnectionCount() const;

  /** Initialize the muxer with the given server socket. The muxer will
   * accept new connections on the server socket and add them to its list
   * of managed connections.
   *
   * @param socket The socket to listen on for new connections. The socket
   * must have already been initialized and put in a listening state.
   * @return <b>true</b> on success, <b>false</b> if the socket is not in
   * a listening state.
   */
  virtual bool init(ServerSocket* socket);

  /** Write a block of data to all active connections.
   *
   * @param buf The buffer containing the data to be sent.
   * @param count The number of elements to write.
   * @return The number of connections to which the data was successfully
   * queued.
   */

  uint_t writeAll(const byte_t *buf, size_t count);

  protected:

  /** This method is called when a new connection is accepted. It must
   * construct and return a new Connection object representing the
   * connection, or <b>NULL</b> if the connection should be denied.
   *
   * @param address The address of the remote peer.
   * @return A new Connection object for this connection, or <b>NULL</b>
   * if the connection should be closed (rejected).
   */

  virtual Connection *connectionReady(const SocketAddress &address) = 0;

  /** This method is called when data has been received on the connection.
   * It is called only when the total number of bytes available to read
   * is at least equal to the value of the read low-water mark for the
   * connection.
   *
   * @param connection The connection.
   */

  virtual void dataReceived(Connection *connection) = 0;

  /** This method is called when the amount of data that is queued to be
   * sent on the connection is less than or equal to the write low-water
   * mark for the connection. This is an indicator to the application that
   * it can now send more data. The default implementation does nothing.
   *
   * @param connection The connection.
   */
  virtual void dataSent(Connection *connection);

  /** This method is called when an out-of-band data byte has been received
   * on the connection. The default implementation does nothing.
   *
   * @param connection The connection.
   */

  virtual void dataReceivedOOB(Connection *connection);

  /** This method is called when a connection is closed, either by
   * request or because the remote peer disconnected. The method
   * should delete the connection object before returning.
   *
   * @param connection The connection.
   */

  virtual void connectionClosed(Connection *connection) = 0;

  /** This method is called when a connection is closed because it
   * timed out due to inactivity. The method should delete the
   * connection object before returning.
   *
   * @param connection The connection.
   */

  virtual void connectionTimedOut(Connection *connection) = 0;

  /** This method is called when an exception occurs during I/O. The
   * default implementation closes the connection.
   *
   * @param connection The connection.
   * @param ex The exception that occurred.
   */

  virtual void exceptionOccurred(Connection *connection,
                                 const IOException& ex);

  class ConnectionList; // fwd decl

  /** The list of active connections. */

  ConnectionList *_connections;

  private:

  void _connectionTimedOut(Connection *connection);
  void _connectionClosed(Connection *connection);

  Mutex _mutex;
  StaticObjectPool<StreamSocket> _pool;
  uint_t _idleLimit;
  uint_t _sleepIntervalSec;
  uint_t _sleepIntervalMicroSec;
  ServerSocket* _ssock;

  CCXX_COPY_DECLS(SocketMuxer);
};

};

#endif // __ccxx_SocketMuxer_hxx

/* end of header file */
