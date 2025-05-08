package org.firstinspires.ftc.teamcode.robochargers;

/**
 * Handy class from this StackOverflow post: https://stackoverflow.com/a/50461711/10803269
 */
public class NotYetImplementedException extends RuntimeException
{
    /**
     * @deprecated Deprecated to remind you to implement the corresponding code
     *             before releasing the software.
     */
    @Deprecated
    public NotYetImplementedException()
    {
    }

    /**
     * @deprecated Deprecated to remind you to implement the corresponding code
     *             before releasing the software.
     */
    @Deprecated
    public NotYetImplementedException(String message)
    {
        super(message);
    }
}