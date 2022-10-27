//! Traits and types for application credentials checkers, used to
//! decide whether an application can be loaded. See
//| the [AppID TRD](../../doc/reference/trd-appid.md).

pub mod basic;

use crate::config;
use crate::debug;
use crate::process::{Process, ShortID, State};
use crate::ErrorCode;
use tock_tbf::types::TbfFooterV2Credentials;

/// What a AppCredentialsChecker decided a particular application's credential
/// indicates about the runnability of an application binary.
#[derive(Debug)]
pub enum CheckResult {
    /// Accept the credential and run the binary.
    Accept,
    /// Go to the next credential or in the case of the last one fall
    /// back to the default policy.
    Pass,
    /// Reject the credential and do not run the binary.
    Reject,
}

/// Receives callbacks on whether a credential was accepted or not.
pub trait Client<'a> {
    fn check_done(
        &self,
        result: Result<CheckResult, ErrorCode>,
        credentials: TbfFooterV2Credentials,
        binary: &'a [u8],
    );
}

/// Implements a Credentials Checking Policy.
pub trait AppCredentialsChecker<'a> {
    fn set_client(&self, _client: &'a dyn Client<'a>);
    fn require_credentials(&self) -> bool;
    fn check_credentials(
        &self,
        credentials: TbfFooterV2Credentials,
        binary: &'a [u8],
    ) -> Result<(), (ErrorCode, TbfFooterV2Credentials, &'a [u8])>;
}

/// Default implementation.
impl<'a> AppCredentialsChecker<'a> for () {
    fn set_client(&self, _client: &'a dyn Client<'a>) {}
    fn require_credentials(&self) -> bool {
        false
    }

    fn check_credentials(
        &self,
        credentials: TbfFooterV2Credentials,
        binary: &'a [u8],
    ) -> Result<(), (ErrorCode, TbfFooterV2Credentials, &'a [u8])> {
        Err((ErrorCode::NOSUPPORT, credentials, binary))
    }
}

/// Return whether `process` can run given the identifiers, version
/// numbers, and execution state of other processes. A process is
/// runnable if:
///
///   - Its Application Identifier and Short ID are different from
///   all other processes, or
///   - Other processes that share an Application Identifier or Short
///   ID with it have a lower version number and are not running, or
///   - Other processes that shared 
pub fn is_runnable<AU: AppUniqueness>(
    process: &dyn Process,
    processes: &[Option<&dyn Process>],
    id_differ: &AU,
) -> bool {
    let len = processes.len();
    // A process is only runnable if it has approved credentials and
    // is not currently running.
    if process.get_state() != State::CredentialsApproved && process.get_state() != State::Terminated
    {
        return false;
    }

    // Note that this causes `process` to compare against itself;
    // however, since `process` is not running and its version number
    // is the same, it will not block itself from running.
    for i in 0..len {
        let other_process = processes[i];
        let other_name = other_process.map_or("None", |c| c.get_process_name());

        let blocks = other_process.map_or(false, |other| {
            let state = other.get_state();
            let creds_approve =
                state != State::CredentialsUnchecked && state != State::CredentialsFailed;
            let different = id_differ.different_identifier(process, other)
                && other.short_app_id() != process.short_app_id();
            let newer = other.binary_version() > process.binary_version();
            let running = other.is_running();
            let runnable = state != State::CredentialsUnchecked
                && state != State::CredentialsFailed
                && state != State::Terminated;
            // Other will block process from running if
            // 1) Other has approved credentials, and
            // 2) Other has the same ShortID or Application Identifier, and
            // 3) Other has a higher version number *or* the same version number and is running
            if config::CONFIG.debug_process_credentials {
                debug!(
                    "[{}]: creds_approve: {}, different: {}, newer: {}, runnable: {}, running: {}",
                    other.get_process_name(),
                    creds_approve,
                    different,
                    newer,
                    runnable,
                    running
                );
            }
            creds_approve && !different && ((newer && runnable) || running)
        });
        if blocks {
            if config::CONFIG.debug_process_credentials {
                debug!(
                    "Process {} blocks {}",
                    other_name,
                    process.get_process_name()
                );
            }
            return false;
        }
    }
    if config::CONFIG.debug_process_credentials {
        debug!(
            "No process blocks {}: it is runnable",
            process.get_process_name()
        );
    }
    // No process blocks this one from running -- it's runnable
    true
}

/// Whether two processes have the same Application Identifier; two
/// processes with the same Application Identifier cannot run concurrently.
pub trait AppUniqueness {
    /// Returns whether `process_a` and `process_b` have a different identifier,
    /// and so can run concurrently. If this returns `false`, the kernel
    /// will not run `process_a` and `process_b` at the same time.
    fn different_identifier(&self, _process_a: &dyn Process, _process_b: &dyn Process) -> bool;
}

/// Default implementation.
impl AppUniqueness for () {
    fn different_identifier(&self, _process_a: &dyn Process, _process_b: &dyn Process) -> bool {
        true
    }
}

/// Transforms Application Credentials into a corresponding ShortID.
pub trait Compress {
    fn to_short_id(&self, _credentials: &TbfFooterV2Credentials) -> ShortID;
}

impl Compress for () {
    fn to_short_id(&self, _credentials: &TbfFooterV2Credentials) -> ShortID {
        ShortID::LocallyUnique
    }
}

pub trait CredentialsCheckingPolicy<'a>:
    AppCredentialsChecker<'a> + Compress + AppUniqueness
{
}
impl<'a, T: AppCredentialsChecker<'a> + Compress + AppUniqueness> CredentialsCheckingPolicy<'a>
    for T
{
}
